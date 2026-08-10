#!/usr/bin/env python3
"""Build the additive 28-muscle lumbar model from existing MyoAssist assets.

This tool treats the healthy 22-muscle baseline and the MyoTorso abdomen donor
as read-only inputs.  It writes only the generated model path unless an
alternative output is explicitly supplied.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
from pathlib import Path
import sys

from lxml import etree
import mujoco


REPO_ROOT = Path(__file__).resolve().parents[1]
BASELINE_PATH = REPO_ROOT / "models/22muscle_2D/myoLeg22_2D_BASELINE.xml"
DONOR_ASSETS_PATH = (
    REPO_ROOT / "myosuite/simhive/myo_sim/torso/assets/myotorso_abdomen_assets.xml"
)
DONOR_CHAIN_PATH = (
    REPO_ROOT / "myosuite/simhive/myo_sim/torso/assets/myotorso_abdomen_chain.xml"
)
DEFAULT_OUTPUT_PATH = (
    REPO_ROOT / "models/22muscle_2D_lumbar/myoLeg28_2D_LUMBAR.xml"
)

LUMBAR_MUSCLES = (
    "ercspn_r",
    "ercspn_l",
    "intobl_r",
    "intobl_l",
    "extobl_r",
    "extobl_l",
)

GENERATED_MARKER = "CODEX_LUMBAR_TRANSPLANT_GENERATED"


def parse_xml(path: Path) -> etree._ElementTree:
    parser = etree.XMLParser(remove_blank_text=False, remove_comments=False)
    return etree.parse(str(path), parser)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def require_one(root: etree._Element, xpath: str, description: str) -> etree._Element:
    matches = root.xpath(xpath)
    if len(matches) != 1:
        raise RuntimeError(
            f"Expected exactly one {description} at {xpath!r}; found {len(matches)}"
        )
    return matches[0]


def insert_before_first_body(parent: etree._Element, element: etree._Element) -> None:
    for index, child in enumerate(parent):
        if child.tag == "body":
            parent.insert(index, element)
            return
    parent.append(element)


def make_joint() -> etree._Element:
    return etree.Element(
        "joint",
        name="lumbar_extension",
        axis="0 0 1",
        pos="0 0 0",
        range="-0.8727 0.2618",
        armature="0.01",
        damping="0.5",
        limited="true",
    )


def make_wrap_geom() -> etree._Element:
    # These explicit values are inherited from the donor's myotorso_wrap class.
    return etree.Element(
        "geom",
        name="pelvis_wrap",
        type="cylinder",
        size="0.05 0.05",
        pos="0 0.0175 0",
        group="3",
        contype="0",
        conaffinity="0",
        rgba="0.5 0.5 0.7 0.2",
    )


def transplant_sites(
    donor_sacrum: etree._Element,
    donor_lumbar: etree._Element,
    pelvis: etree._Element,
    torso: etree._Element,
) -> None:
    for muscle in LUMBAR_MUSCLES:
        p1 = require_one(
            donor_sacrum,
            f"./site[@name='{muscle}_{muscle}-P1']",
            f"{muscle} pelvis attachment",
        )
        p2 = require_one(
            donor_lumbar,
            f"./site[@name='{muscle}_{muscle}-P2']",
            f"{muscle} torso attachment",
        )
        for donor_site, target in ((p1, pelvis), (p2, torso)):
            site = copy.deepcopy(donor_site)
            site.attrib.pop("class", None)
            site.set("size", "0.001 0.005 0.005")
            site.set("group", "3")
            insert_before_first_body(target, site)


def transplant_tendons_and_actuators(
    donor_assets: etree._Element,
    target_root: etree._Element,
) -> None:
    target_tendons = require_one(target_root, "./tendon", "target tendon section")
    target_actuators = require_one(target_root, "./actuator", "target actuator section")

    donor_tendons = require_one(donor_assets, "./tendon", "donor tendon section")
    donor_actuators = require_one(donor_assets, "./actuator", "donor actuator section")

    for muscle in LUMBAR_MUSCLES:
        tendon = require_one(
            donor_tendons,
            f"./spatial[@name='{muscle}_tendon']",
            f"{muscle} tendon",
        )
        tendon_copy = copy.deepcopy(tendon)
        tendon_copy.attrib.pop("class", None)
        tendon_copy.set("width", "0.001")
        tendon_copy.set("rgba", "0.95 0.3 0.3 1")
        target_tendons.append(tendon_copy)

        actuator = require_one(
            donor_actuators,
            f"./general[@name='{muscle}']",
            f"{muscle} actuator",
        )
        actuator_copy = copy.deepcopy(actuator)
        # The baseline's muscle default is numerically identical to the donor's
        # myotorso_muscle default, so this keeps all inherited parameters intact.
        actuator_copy.set("class", "muscle")
        target_actuators.append(actuator_copy)


def add_disabled_neutral_lock(target_root: etree._Element) -> None:
    """Add an opt-in equality used only for neutral-lumbar physics checks."""
    equality = require_one(target_root, "./equality", "target equality section")
    equality.append(
        etree.Element(
            "joint",
            name="lumbar_neutral_lock",
            joint1="lumbar_extension",
            polycoef="0 0 0 0 0",
            active="false",
            solref="0.001 1",
            solimp="0.9999 0.9999 0.001 0.5 2",
        )
    )


def insert_keyframe_coordinate(root: etree._Element, qpos_index: int, qvel_index: int) -> None:
    keys = root.xpath("./keyframe/key")
    if not keys:
        raise RuntimeError("The baseline contains no keyframes to update")

    for key in keys:
        for attribute, index in (("qpos", qpos_index), ("qvel", qvel_index)):
            values = key.get(attribute, "").split()
            if not values:
                raise RuntimeError(
                    f"Keyframe {key.get('name')!r} has no {attribute} values"
                )
            values.insert(index, "0")
            key.set(attribute, " ".join(values))


def build_model_bytes() -> bytes:
    baseline_tree = parse_xml(BASELINE_PATH)
    donor_assets_tree = parse_xml(DONOR_ASSETS_PATH)
    donor_chain_tree = parse_xml(DONOR_CHAIN_PATH)

    root = baseline_tree.getroot()
    donor_assets = donor_assets_tree.getroot()
    donor_chain = donor_chain_tree.getroot()

    if root.xpath(".//joint[@name='lumbar_extension']"):
        raise RuntimeError("The baseline unexpectedly already contains lumbar_extension")

    pelvis = require_one(root, ".//body[@name='pelvis']", "baseline pelvis body")
    torso = require_one(root, ".//body[@name='torso']", "baseline torso body")
    donor_sacrum = require_one(
        donor_chain, "./body[@name='sacrum']", "donor sacrum body"
    )
    donor_lumbar = require_one(
        donor_sacrum, "./body[@name='lumbar']", "donor lumbar body"
    )

    if torso.get("pos") != donor_lumbar.get("pos"):
        raise RuntimeError(
            "The donor lumbar and baseline torso origins no longer match: "
            f"{donor_lumbar.get('pos')} != {torso.get('pos')}"
        )

    baseline_model = mujoco.MjModel.from_xml_path(str(BASELINE_PATH))
    first_torso_joint = mujoco.mj_name2id(
        baseline_model, mujoco.mjtObj.mjOBJ_JOINT, "r_shoulder_abd"
    )
    if first_torso_joint < 0:
        raise RuntimeError("Could not locate r_shoulder_abd in the baseline")
    qpos_index = int(baseline_model.jnt_qposadr[first_torso_joint])
    qvel_index = int(baseline_model.jnt_dofadr[first_torso_joint])

    root.set("model", "myoLeg28_2D_LUMBAR")
    root.insert(
        0,
        etree.Comment(
            f" {GENERATED_MARKER}: additive copy of "
            "models/22muscle_2D/myoLeg22_2D_BASELINE.xml with six muscles "
            "from myotorso_abdomen; source files are not modified. "
        ),
    )
    root.insert(
        1,
        etree.Comment(
            f" baseline_sha256={sha256(BASELINE_PATH)} "
            f"donor_assets_sha256={sha256(DONOR_ASSETS_PATH)} "
            f"donor_chain_sha256={sha256(DONOR_CHAIN_PATH)} "
        ),
    )

    # A joint belongs to the torso body, so it articulates the existing torso
    # and all arm/head descendants relative to the unchanged pelvis.
    inertial = require_one(torso, "./inertial", "baseline torso inertial")
    torso.insert(torso.index(inertial) + 1, make_joint())
    insert_before_first_body(torso, make_wrap_geom())
    transplant_sites(donor_sacrum, donor_lumbar, pelvis, torso)
    transplant_tendons_and_actuators(donor_assets, root)
    add_disabled_neutral_lock(root)
    insert_keyframe_coordinate(root, qpos_index, qvel_index)

    etree.indent(baseline_tree, space="    ")
    return etree.tostring(
        baseline_tree,
        encoding="UTF-8",
        xml_declaration=True,
        pretty_print=True,
    )


def write_model(output: Path, replace_generated: bool) -> None:
    output = output.resolve()
    protected = {
        BASELINE_PATH.resolve(),
        DONOR_ASSETS_PATH.resolve(),
        DONOR_CHAIN_PATH.resolve(),
    }
    if output in protected:
        raise RuntimeError(f"Refusing to overwrite protected source model: {output}")

    payload = build_model_bytes()
    if output.exists():
        existing = output.read_bytes()
        if existing == payload:
            print(f"Integrated model is already up to date: {output}")
            return
        if GENERATED_MARKER.encode() not in existing or not replace_generated:
            raise RuntimeError(
                f"Refusing to replace existing file: {output}. "
                "Use --replace-generated only for a previously generated model."
            )

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(payload)
    print(f"Wrote additive lumbar model: {output}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT_PATH,
        help=f"Generated XML path (default: {DEFAULT_OUTPUT_PATH})",
    )
    parser.add_argument(
        "--replace-generated",
        action="store_true",
        help="Replace only an existing file carrying the generated-model marker.",
    )
    args = parser.parse_args()

    try:
        write_model(args.output, args.replace_generated)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
