#!/usr/bin/env python3
"""Generate the dual-Milly URDF from the canonical single-arm description."""

from __future__ import annotations

import argparse
import copy
import hashlib
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

DESCRIPTION_DIR = Path(__file__).resolve().parent
SOURCE = DESCRIPTION_DIR.parent / "milly" / "urdf" / "milly.urdf"
OUTPUT = DESCRIPTION_DIR / "urdf" / "milly-dual.urdf"
BELLO_NS = "https://motomind.kr/bello/urdf"
ET.register_namespace("bello", BELLO_NS)
# Left and right are from the operator's head-on view.
SIDES = (("left", -0.2215), ("right", 0.2215))
MOUNT_RPY = "0 0 1.5707963267948966"
CENTER_D405_LONGITUDINAL_X = 0.040
CENTER_D405_LATERAL_Y = -0.009
CENTER_D405_HEIGHT = 0.35937731
CENTER_D405_PITCH = 0.44660681555959347
CENTER_D405_YAW = 3.141592653589793
CENTER_D405_SIZE = (0.023, 0.042, 0.042)
CENTER_D405_KEEPOUT_SIZE = (0.100, 0.100, 0.100)
CENTER_D405_SUPPORT_POST_SIZE_XY = 0.040
CENTER_D405_SUPPORT_POST_HEIGHT = CENTER_D405_HEIGHT - CENTER_D405_KEEPOUT_SIZE[2] / 2.0
CENTER_D405_SUPPORT_POST_CENTER_Z = CENTER_D405_SUPPORT_POST_HEIGHT / 2.0
OPTICAL_FRAME_RPY = "-1.5707963267948966 0 -1.5707963267948966"


def _prefix_names(element: ET.Element, side: str) -> None:
    prefix = f"{side}_"
    for node in element.iter():
        if "name" in node.attrib:
            node.set("name", prefix + node.get("name", ""))
        if node.tag in {"parent", "child"} and "link" in node.attrib:
            node.set("link", prefix + node.get("link", ""))
        if node.tag == "mimic" and "joint" in node.attrib:
            node.set("joint", prefix + node.get("joint", ""))
        if node.tag == f"{{{BELLO_NS}}}slider_crank":
            for attribute in ("driver", "sliders", "rods"):
                node.set(
                    attribute,
                    " ".join(prefix + name for name in node.attrib[attribute].split()),
                )


def _rewrite_mesh_paths(element: ET.Element) -> None:
    for mesh in element.iter("mesh"):
        filename = mesh.get("filename")
        if filename is None or filename.startswith("package://"):
            continue
        source_mesh = (SOURCE.parent / filename).resolve()
        relative = os.path.relpath(source_mesh, OUTPUT.parent)
        mesh.set("filename", Path(relative).as_posix())


def _clone(element: ET.Element, side: str) -> ET.Element:
    cloned = copy.deepcopy(element)
    _prefix_names(cloned, side)
    _rewrite_mesh_paths(cloned)
    return cloned


def _mount(side: str, y: float) -> ET.Element:
    joint = ET.Element("joint", {"name": f"{side}_mount", "type": "fixed"})
    ET.SubElement(joint, "parent", {"link": "dual_milly_base"})
    ET.SubElement(joint, "child", {"link": f"{side}_base_link"})
    ET.SubElement(joint, "origin", {"xyz": f"0 {y:.4f} 0", "rpy": MOUNT_RPY})
    return joint


def _center_d405() -> tuple[ET.Element, ...]:
    body = ET.Element("link", {"name": "center_d405_link"})
    visual = ET.SubElement(body, "visual")
    ET.SubElement(visual, "origin", {"xyz": "-0.0115 0 0", "rpy": "0 0 0"})
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(
        geometry,
        "box",
        {"size": " ".join(f"{value:.3f}" for value in CENTER_D405_SIZE)},
    )
    material = ET.SubElement(visual, "material", {"name": "center_d405_body"})
    ET.SubElement(material, "color", {"rgba": "0.12 0.12 0.12 1"})

    optical = ET.Element("link", {"name": "center_d405_color_optical_frame"})

    mount = ET.Element("joint", {"name": "center_d405_mount", "type": "fixed"})
    ET.SubElement(mount, "parent", {"link": "dual_milly_base"})
    ET.SubElement(mount, "child", {"link": "center_d405_link"})
    ET.SubElement(
        mount,
        "origin",
        {
            "xyz": (
                f"{CENTER_D405_LONGITUDINAL_X:.3f} "
                f"{CENTER_D405_LATERAL_Y:.3f} {CENTER_D405_HEIGHT:.8f}"
            ),
            "rpy": f"0 {CENTER_D405_PITCH:.17g} {CENTER_D405_YAW:.17g}",
        },
    )

    optical_joint = ET.Element(
        "joint", {"name": "center_d405_color_optical_joint", "type": "fixed"}
    )
    ET.SubElement(optical_joint, "parent", {"link": "center_d405_link"})
    ET.SubElement(optical_joint, "child", {"link": "center_d405_color_optical_frame"})
    ET.SubElement(optical_joint, "origin", {"xyz": "0 0 0", "rpy": OPTICAL_FRAME_RPY})
    return body, optical, mount, optical_joint


def _fixed_collision_box(
    *,
    link_name: str,
    joint_name: str,
    collision_name: str,
    center: tuple[float, float, float],
    size: tuple[float, float, float],
) -> tuple[ET.Element, ET.Element]:
    link = ET.Element("link", {"name": link_name})
    collision = ET.SubElement(link, "collision", {"name": collision_name})
    ET.SubElement(collision, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    geometry = ET.SubElement(collision, "geometry")
    ET.SubElement(
        geometry,
        "box",
        {"size": " ".join(f"{value:.9g}" for value in size)},
    )

    joint = ET.Element("joint", {"name": joint_name, "type": "fixed"})
    ET.SubElement(joint, "parent", {"link": "dual_milly_base"})
    ET.SubElement(joint, "child", {"link": link_name})
    ET.SubElement(
        joint,
        "origin",
        {
            "xyz": " ".join(f"{value:.9g}" for value in center),
            "rpy": "0 0 0",
        },
    )
    return link, joint


def _center_d405_collision_obstacles() -> tuple[ET.Element, ...]:
    keepout = _fixed_collision_box(
        link_name="center_d405_keepout_link",
        joint_name="center_d405_keepout_joint",
        collision_name="center_d405_keepout_collision",
        center=(
            CENTER_D405_LONGITUDINAL_X,
            CENTER_D405_LATERAL_Y,
            CENTER_D405_HEIGHT,
        ),
        size=CENTER_D405_KEEPOUT_SIZE,
    )
    support_post = _fixed_collision_box(
        link_name="center_d405_support_post_link",
        joint_name="center_d405_support_post_joint",
        collision_name="center_d405_support_post_collision",
        center=(
            CENTER_D405_LONGITUDINAL_X,
            CENTER_D405_LATERAL_Y,
            CENTER_D405_SUPPORT_POST_CENTER_Z,
        ),
        size=(
            CENTER_D405_SUPPORT_POST_SIZE_XY,
            CENTER_D405_SUPPORT_POST_SIZE_XY,
            CENTER_D405_SUPPORT_POST_HEIGHT,
        ),
    )
    return (*keepout, *support_post)


def render() -> str:
    source_bytes = SOURCE.read_bytes()
    source = ET.fromstring(source_bytes)
    if source.tag != "robot":
        raise ValueError(f"{SOURCE} does not contain a URDF robot")

    robot = ET.Element("robot", {"name": "milly_dual_control"})
    digest = hashlib.sha256(source_bytes).hexdigest()
    robot.append(
        ET.Comment(
            " Generated by descriptions/milly-dual/generate_urdf.py from "
            f"descriptions/milly/urdf/milly.urdf (sha256 {digest}). "
            "Do not edit this file directly. "
        )
    )

    for side, _ in SIDES:
        for material in source.findall("material"):
            robot.append(_clone(material, side))

    robot.append(ET.Element("link", {"name": "dual_milly_base"}))
    robot.extend(_center_d405_collision_obstacles())
    robot.extend(_center_d405())
    for side, y in SIDES:
        for link in source.findall("link"):
            robot.append(_clone(link, side))
        robot.append(_mount(side, y))
        for joint in source.findall("joint"):
            robot.append(_clone(joint, side))
        for transmission in source.findall("transmission"):
            robot.append(_clone(transmission, side))
        for linkage in source.findall(f"{{{BELLO_NS}}}slider_crank"):
            robot.append(_clone(linkage, side))

    ET.indent(robot, space="  ")
    return ET.tostring(robot, encoding="utf-8", xml_declaration=True).decode("utf-8") + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--check",
        action="store_true",
        help="fail if the checked-in dual URDF is not up to date",
    )
    args = parser.parse_args()
    generated = render()
    if args.check:
        if not OUTPUT.is_file() or OUTPUT.read_text() != generated:
            print(f"{OUTPUT} is stale; run {Path(__file__).name}", file=sys.stderr)
            return 1
        return 0
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(generated)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
