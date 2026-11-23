#!/usr/bin/env python3
"""Regenerate Milly's exterior visuals and gripper tree from the case assembly STEP.

uv run --with cadquery==2.8.0 --with trimesh==5.1.1 --with scipy \
    scripts/update-milly-assets.py --step '/path/Milly_Assem_0914_Case Only.STEP'

The input CAD is proprietary and supplied separately.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import cadquery as cq
import numpy as np
import trimesh
from milly_collisions import apply_collisions
from milly_mass_properties import apply_mass_properties
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from scipy.spatial.transform import Rotation

NS = "https://motomind.kr/bello/urdf"
ET.register_namespace("bello", NS)
MATERIALS = {
    "milly_white": "0.88 0.90 0.89 1",
    "milly_black": "0.055 0.060 0.065 1",
    "milly_base": "0.26 0.28 0.29 1",
    "milly_metal": "0.52 0.55 0.58 1",
}
RADIUS, ROD, OFFSET = 0.030, 0.045, 0.014
# The STEP pin centers give the opening angle. Calibrate the encoder phase on hardware.
OPEN_ANGLE = math.atan2(251.2017 - 238.1795, 27.0263)
OPEN_MOTOR = -2.2
PHASE = OPEN_ANGLE - OPEN_MOTOR
SLIDER_REFERENCE = RADIUS * math.cos(OPEN_ANGLE) + math.sqrt(
    ROD**2 - (OFFSET - RADIUS * math.sin(OPEN_ANGLE)) ** 2
)
CENTRE_CAD = np.array([0.0, -0.1985206, 0.2381795])
CAD_FROM_GRIPPER = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])


def pose(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    result[:3, 3] = xyz
    return result


def numbers(values):
    return " ".join(f"{v:.12g}" for v in values)


def origin(parent, transform):
    ET.SubElement(
        parent,
        "origin",
        xyz=numbers(transform[:3, 3]),
        rpy=numbers(Rotation.from_matrix(transform[:3, :3]).as_euler("xyz")),
    )


def read_step(path):
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    if reader.ReadFile(str(path)) != IFSelect_RetDone:
        raise ValueError(f"Cannot read STEP: {path}")
    doc = TDocStd_Document(TCollection_ExtendedString("XmXCAF"))
    if not reader.Transfer(doc):
        raise ValueError("Cannot transfer STEP assembly")
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    roots = TDF_LabelSequence()
    tool.GetFreeShapes(roots)
    parts = []

    def walk(label, location, path):
        location = location * cq.Location(tool.GetLocation_s(label))
        ref = TDF_Label()
        if tool.IsReference_s(label):
            tool.GetReferredShape_s(label, ref)
        else:
            ref = label
        attr = TDataStd_Name()
        ref.FindAttribute(TDataStd_Name.GetID_s(), attr)
        name = attr.Get().ToExtString()
        path = path + [f"{label.Tag():02d}_{name}"]
        children = TDF_LabelSequence()
        tool.GetComponents_s(ref, children)
        if children.Length():
            for child in children:
                walk(child, location, path)
        else:
            shape = cq.Shape.cast(tool.GetShape_s(ref)).located(location)
            points, faces = shape.tessellate(0.10, 0.15)
            mesh = trimesh.Trimesh(
                np.array([p.toTuple() for p in points]) * 0.001, faces, process=True
            )
            parts.append({"path": "/".join(path), "name": name, "mesh": mesh})

    for label in roots:
        walk(label, cq.Location(), [])
    return parts


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--step", type=Path, required=True)
    parser.add_argument(
        "--description",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "descriptions/milly",
    )
    args = parser.parse_args()
    dest = args.description.resolve()
    urdf = dest / "urdf/milly.urdf"
    root = ET.parse(urdf).getroot()
    parts = read_step(args.step)
    expected = [
        "AL1_J1_TOP_BRKT",
        "Base_Mount",
        "Base",
        "PLA1_Motor03_SPACER",
        "AL4_J2_TOP_BRKT",
        "AL5_J2_INNER_FRAME",
        "Part1",
        "Part1.1",
        "PLA2_Motor06_SPACER",
        "AL8_J3_TOP_BRKT",
        "AL9_J2_INNER_FRAME_R",
        "AL10_J2_INNER_FRAME_L",
        "Part2",
        "Part2.1",
        "Motor00",
        "AL12_J4_BTM_OUTER_LIMITER",
        "AL13_J4_TOP_BRKT",
        "AL14_J5_INNER_FRAME",
        "Motor00",
        "PLA3_Motor00_SPACER",
        "AL19_J5_TOP_BRKT.1",
        "Motor05",
        "AL15_J5_OUTER_FRAME",
        "Arm_End_Plate",
        "Axis7_Limit_Frame_H",
        "Part3",
        "Motor_00MINI.2",
        "Finger",
        "Gripper Finger Bracket",
        "Linear guide block",
        "Gripper_SwingArm",
        "Linear rail",
        "Finger",
        "Gripper Finger Bracket",
        "Linear guide block",
        "Gripper_SwingArm",
        "Part1.1.2.3",
        "Part1.1.2",
        "USB Camera",
    ]
    if [p["name"] for p in parts] != expected:
        raise ValueError(
            "STEP assembly differs from the reviewed 0914 case-only assembly"
        )

    links = {link.get("name"): link for link in root.findall("link")}
    # Align the CAD shoulder axes to the URDF shoulder frame.
    transforms = {"link_1": pose()}
    transforms["link_1"][:3, :3] = np.diag([-1.0, 1.0, -1.0])
    transforms["link_1"][:3, 3] = [0.0, 0.0575, 0.072]
    joints = {joint.get("name"): joint for joint in root.findall("joint")}
    for i in range(1, 7):
        joint = joints[f"joint_{i}"]
        o = joint.find("origin")
        joint_pose = pose(
            list(map(float, o.get("xyz").split())),
            list(map(float, o.get("rpy").split())),
        )
        if i == 1:
            transforms["base_link"] = joint_pose @ transforms["link_1"]
        else:
            transforms[f"link_{i}"] = (
                np.linalg.inv(joint_pose) @ transforms[f"link_{i - 1}"]
            )

    for node in list(root):
        if (
            node.tag == "material"
            or node.tag.startswith("{")
            or node.tag == "joint"
            and node.get("name") not in {f"joint_{i}" for i in range(1, 7)}
            or node.tag == "link"
            and node.get("name") not in transforms
        ):
            root.remove(node)
    for link in root.findall("link"):
        for visual in link.findall("visual"):
            link.remove(visual)
        if link.get("name") == "link_6":
            for collision in link.findall("collision"):
                link.remove(collision)
    for name, rgba in MATERIALS.items():
        node = ET.Element("material", name=name)
        ET.SubElement(node, "color", rgba=rgba)
        root.insert(0, node)

    def new_link(name):
        links[name] = ET.SubElement(root, "link", name=name)
        return links[name]

    def joint(name, parent, child, transform, kind="fixed", axis=None, limits=None):
        node = ET.SubElement(root, "joint", name=name, type=kind)
        ET.SubElement(node, "parent", link=parent)
        ET.SubElement(node, "child", link=child)
        origin(node, transform)
        if axis is not None:
            ET.SubElement(node, "axis", xyz=numbers(axis))
        if limits:
            ET.SubElement(
                node,
                "limit",
                lower=str(limits[0]),
                upper=str(limits[1]),
                effort=str(limits[2]),
                velocity=str(limits[3]),
            )
        return node

    gripper_from_cad = pose()
    gripper_from_cad[:3, :3] = CAD_FROM_GRIPPER.T
    gripper_from_cad[:3, 3] = -CAD_FROM_GRIPPER.T @ CENTRE_CAD
    new_link("gripper_base")
    transforms["gripper_base"] = gripper_from_cad
    joint(
        "gripper_base_mount",
        "link_6",
        "gripper_base",
        transforms["link_6"] @ np.linalg.inv(gripper_from_cad),
    )
    new_link("gripper_drive")
    transforms["gripper_drive"] = pose(rpy=(0, 0, -OPEN_MOTOR)) @ gripper_from_cad
    joint(
        "gripper",
        "gripper_base",
        "gripper_drive",
        pose(),
        "revolute",
        (0, 0, 1),
        (-2.4, 0.1, 20, 20),
    )
    beta_open = math.asin((OFFSET - RADIUS * math.sin(OPEN_ANGLE)) / ROD)
    for i, sign in [(1, 1), (2, -1)]:
        name = f"finger_{i}"
        new_link(name)
        slider_pose = pose((sign * SLIDER_REFERENCE, 0, 0))
        transforms[name] = np.linalg.inv(slider_pose) @ gripper_from_cad
        joint(
            f"finger_{i}_joint",
            "gripper_base",
            name,
            slider_pose,
            "prismatic",
            (sign, 0, 0),
            (-0.065, 0.005, 100, 2),
        )
        rod_name = f"gripper_rod_{i}"
        new_link(rod_name)
        open_pin = (
            sign * RADIUS * math.cos(OPEN_ANGLE),
            sign * RADIUS * math.sin(OPEN_ANGLE),
            0,
        )
        rod_open = pose(open_pin, (0, 0, beta_open + (math.pi if sign < 0 else 0)))
        transforms[rod_name] = np.linalg.inv(rod_open) @ gripper_from_cad
        pin = (sign * RADIUS * math.cos(PHASE), sign * RADIUS * math.sin(PHASE), 0)
        rod_origin = pose(pin, (0, 0, PHASE + (math.pi if sign < 0 else 0)))
        joint(
            f"gripper_rod_{i}_joint",
            "gripper_drive",
            rod_name,
            rod_origin,
            "revolute",
            (0, 0, 1),
            (-4, 1, 100, 100),
        )
        new_link(f"gripper_rod_{i}_tip")
        joint(
            f"gripper_rod_{i}_tip_mount",
            rod_name,
            f"gripper_rod_{i}_tip",
            pose((ROD, 0, 0.004025)),
        )
        new_link(f"finger_{i}_pin")
        joint(
            f"finger_{i}_pin_mount",
            name,
            f"finger_{i}_pin",
            pose((0, sign * OFFSET, 0.004025)),
        )

    new_link("camera_link")
    lens = np.array([0.0, -0.1667094, 0.3148766])
    cad_from_camera = pose(lens)
    # REP-103 body frame: X forward along the lens, Y left, Z up.
    cad_from_camera[:3, :3] = np.column_stack(
        ([0.0, -math.sqrt(3) / 2, -0.5], [1.0, 0.0, 0.0], [0.0, -0.5, math.sqrt(3) / 2])
    )
    transforms["camera_link"] = np.linalg.inv(cad_from_camera)
    joint(
        "camera_mount", "link_6", "camera_link", transforms["link_6"] @ cad_from_camera
    )
    new_link("camera_optical_frame")
    joint(
        "camera_optical_joint",
        "camera_link",
        "camera_optical_frame",
        pose(rpy=(-math.pi / 2, 0, -math.pi / 2)),
    )

    ET.SubElement(
        root,
        f"{{{NS}}}slider_crank",
        name="milly_gripper",
        driver="gripper",
        sliders="finger_1_joint finger_2_joint",
        rods="gripper_rod_1_joint gripper_rod_2_joint",
        crank_radius=str(RADIUS),
        rod_length=str(ROD),
        slider_offset=str(OFFSET),
        phase=f"{PHASE:.12g}",
        slider_reference=f"{SLIDER_REFERENCE:.12g}",
    )

    visual_dir = dest / "meshes/visual/exterior"
    visual_dir.mkdir(parents=True, exist_ok=True)
    groups = [
        ("base_link", "milly_base", [1, 2], "base"),
        ("link_1", "milly_black", [5], "shoulder_frame"),
        ("link_2", "milly_black", [3], "shoulder_shroud"),
        ("link_2", "milly_white", [6, 7], "upper_arm_case"),
        ("link_3", "milly_black", [8, 10, 11], "elbow_frame"),
        ("link_3", "milly_white", [12, 13], "forearm_case"),
        ("link_4", "milly_black", [14, 15, 17], "wrist_roll"),
        ("link_5", "milly_black", [18, 21, 23], "wrist_pitch"),
        ("link_5", "milly_white", [25], "wrist_case"),
        ("gripper_base", "milly_black", [26, 36], "gripper_body"),
        ("gripper_base", "milly_metal", [31], "gripper_rail"),
        ("gripper_drive", "milly_metal", [37], "gripper_crank"),
        ("finger_1", "milly_metal", [34], "finger_1_carriage"),
        ("finger_2", "milly_metal", [29], "finger_2_carriage"),
        ("gripper_rod_1", "milly_metal", [35], "gripper_rod_1"),
        ("gripper_rod_2", "milly_metal", [30], "gripper_rod_2"),
        ("camera_link", "milly_black", [38], "camera"),
    ]
    manifest = []
    used = set()
    for link, material, indices, filename in groups:
        mesh = trimesh.util.concatenate([parts[i]["mesh"] for i in indices])
        mesh.apply_transform(transforms[link])
        mesh.export(visual_dir / f"{filename}.stl")
        visual = ET.SubElement(links[link], "visual", name=filename)
        geometry = ET.SubElement(visual, "geometry")
        ET.SubElement(
            geometry, "mesh", filename=f"../meshes/visual/exterior/{filename}.stl"
        )
        ET.SubElement(visual, "material", name=material)
        used.update(indices)
        manifest.append(
            {
                "mesh": filename + ".stl",
                "link": link,
                "material": material,
                "instances": [parts[i]["path"] for i in indices],
                "triangles": len(mesh.faces),
            }
        )

    # The case assembly omits the installed base connector and uses different jaws.
    retained_dir = dest / "meshes/visual/retained"
    retained_dir.mkdir(exist_ok=True)
    legacy = [
        (
            "base_link",
            "base_connector",
            "base_link__001__Connector_Cnlinko_LP-20_4_poles.1.stl",
            pose(),
        ),
    ]
    for i, xyz in [
        (1, [0.06552811, 0.0081136605, -0.11519444]),
        (2, [-0.065921114, -0.0042637112, -0.11519444]),
    ]:
        # The jaw meshes are in link_6 coordinates.
        relative = (
            transforms[f"finger_{i}"] @ np.linalg.inv(transforms["link_6"]) @ pose(xyz)
        )
        legacy.append(
            (
                f"finger_{i}",
                f"finger_{i}",
                f"finger_{i}__000__Gripper_Finger.{i}.stl",
                relative,
            )
        )
    retained_manifest = []
    for link, name, original, relative in legacy:
        source = dest / "meshes/visual/parts" / original
        target = retained_dir / f"{name}.stl"
        if source.exists():
            shutil.copyfile(source, target)
        if not target.exists():
            raise ValueError(f"Missing retained legacy mesh: {target}")
        mesh = trimesh.load_mesh(target)
        mesh.apply_transform(relative)
        visual = ET.SubElement(links[link], "visual", name=name)
        origin(visual, relative)
        geometry = ET.SubElement(visual, "geometry")
        ET.SubElement(
            geometry, "mesh", filename=f"../meshes/visual/retained/{name}.stl"
        )
        ET.SubElement(visual, "material", name="milly_black")
        retained_manifest.append(
            {
                "mesh": f"{name}.stl",
                "source_mesh": original,
                "link": link,
                "mesh_to_link": relative.tolist(),
                "sha256": hashlib.sha256(target.read_bytes()).hexdigest(),
            }
        )

    apply_collisions(root, dest)
    apply_mass_properties(root, dest, transforms)
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(urdf, encoding="utf-8", xml_declaration=True)
    with urdf.open("a") as f:
        f.write("\n")

    metadata = {
        "source_file": args.step.name,
        "source_sha256": hashlib.sha256(args.step.read_bytes()).hexdigest(),
        "source_units": "mm",
        "output_units": "m",
        "tessellation": {"linear_deflection_mm": 0.10, "angular_deflection_rad": 0.15},
        "cad_to_link": {name: matrix.tolist() for name, matrix in transforms.items()},
        "visuals": manifest,
        "omitted_instances": [
            parts[i]["path"] for i in range(len(parts)) if i not in used
        ],
        "retained_legacy_meshes": retained_manifest,
        "gripper": {
            "crank_radius_m": RADIUS,
            "rod_length_m": ROD,
            "slider_offset_m": OFFSET,
            "cad_crank_angle_rad": OPEN_ANGLE,
            "motor_at_cad_pose_rad": OPEN_MOTOR,
            "encoder_phase_status": "Requires physical calibration at the open position.",
        },
        "camera": {
            "lens_centre_cad_m": lens.tolist(),
            "optical_forward_cad": [0, -math.sqrt(3) / 2, -0.5],
            "status": "Nominal CAD lens frame; not a measured optical calibration.",
        },
    }
    (dest / "exterior-assets.json").write_text(json.dumps(metadata, indent=2) + "\n")
    old_parts = dest / "meshes/visual/parts"
    if old_parts.exists():
        for old in old_parts.glob("*.stl"):
            old.unlink()
        old_parts.rmdir()
    for replaced in ["base_connector.stl", "finger_1.stl", "finger_2.stl"]:
        (visual_dir / replaced).unlink(missing_ok=True)
    print(
        f"Wrote {len(groups) + len(legacy)} exterior meshes from {len(used)} STEP instances and {len(legacy)} retained meshes."
    )


if __name__ == "__main__":
    main()
