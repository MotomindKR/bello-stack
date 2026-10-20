#!/usr/bin/env python3
"""Mount articulated Milly grippers and wrist cameras on Mini Torso.

uv run --with numpy --with scipy --with trimesh scripts/mini_torso_grippers.py
The direct mounts and SDK wrist aggregate used for mass are provisional.
"""

import math
import xml.etree.ElementTree as ET
from copy import deepcopy
from pathlib import Path

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
DESCRIPTION = ROOT / "descriptions/bello_mini_torso"
URDF = DESCRIPTION / "urdf/bello_mini_torso.urdf"
ATTACHMENT_Z = -0.037497803653348316
HOUSING_REAR_Z = -0.05525
TCP_Z = 0.0795
BELLO_NAMESPACE = "https://motomind.kr/bello/urdf"
ET.register_namespace("bello", BELLO_NAMESPACE)


def numbers(values):
    return " ".join(f"{v:.15g}" for v in values)


def transform(origin):
    frame = np.eye(4)
    if origin is not None:
        frame[:3, 3] = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
        frame[:3, :3] = Rotation.from_euler(
            "xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
        ).as_matrix()
    return frame


def set_origin(node, frame):
    origin = node.find("origin")
    if origin is None:
        origin = ET.SubElement(node, "origin")
    origin.set("xyz", numbers(frame[:3, 3]))
    origin.set("rpy", numbers(Rotation.from_matrix(frame[:3, :3]).as_euler("xyz")))


def main():
    source = ET.parse(ROOT / "descriptions/milly/urdf/milly.urdf").getroot()
    root = ET.parse(URDF).getroot()
    linkage = source.find(f"{{{BELLO_NAMESPACE}}}slider_crank")
    if linkage is None:
        raise ValueError("Milly description is missing its slider-crank linkage")
    # Copy the gripper tree and the wrist-fixed camera, preserving their registration.
    links = {"gripper_base"}
    joints = []
    for joint in source.findall("joint"):
        if joint.find("parent").get("link") in links:
            joints.append(joint)
            links.add(joint.find("child").get("link"))
    camera = deepcopy(source.find("joint[@name='camera_mount']"))
    mount = transform(source.find("joint[@name='gripper_base_mount']/origin"))
    set_origin(camera, np.linalg.inv(mount) @ transform(camera.find("origin")))
    camera.find("parent").set("link", "gripper_base")
    joints.extend([camera, source.find("joint[@name='camera_optical_joint']")])
    links.update(["camera_link", "camera_optical_frame"])
    meshes = DESCRIPTION / "meshes"
    meshes.mkdir(exist_ok=True)
    for side in ("left", "right"):
        prefix = side + "_"
        for node in list(root):
            name = node.get("name", "")
            if name.startswith(prefix) and any(
                token in name
                for token in ("gripper", "finger_", "camera_", "end_effector_sphere")
            ):
                root.remove(node)
        # Original wrist GLBs include entire hands. Retain named wrist components only.
        scene = trimesh.load(
            ROOT / f"descriptions/bello_mini/meshes/{side}_wrist_roll_link.glb"
        )
        keep = (
            "Waist_Outer_Frame",
            "Wrist_Case_",
            "AL3_J2_J3_BTM_BRKT",
            "AL19_J5_TOP_BRKT",
        )
        scene.delete_geometry(
            [name for name in scene.geometry if not any(k in name for k in keep)]
        )
        assert len(scene.geometry) >= 3, "missing wrist exterior components"
        scene.export(meshes / f"{side}_wrist_roll.glb")
        scene.to_mesh().export(meshes / f"{side}_wrist_roll.stl")
        wrist = root.find(f"link[@name='{side}_wrist_roll_link']")
        wrist.find("visual/geometry/mesh").set(
            "filename", f"../meshes/{side}_wrist_roll.glb"
        )
        for name in sorted(links):
            link = deepcopy(source.find(f"link[@name='{name}']"))
            link.set("name", prefix + name)
            for node in link.iter():
                if (
                    node is not link
                    and "name" in node.attrib
                    and node.tag != "material"
                ):
                    node.set("name", prefix + node.get("name"))
                if node.tag == "mesh":
                    node.set("filename", "../../milly/" + node.get("filename")[3:])
            if name == "gripper_base":
                # Milly has no isolated body inertia: explicitly use its residual SDK
                # wrist/camera aggregate in the gripper frame as a mass proxy.
                inertia = deepcopy(source.find("link[@name='link_6']/inertial"))
                mount = transform(
                    source.find("joint[@name='gripper_base_mount']/origin")
                )
                set_origin(
                    inertia, np.linalg.inv(mount) @ transform(inertia.find("origin"))
                )
                link.insert(0, inertia)
            root.append(link)
        for source_joint in joints:
            joint = deepcopy(source_joint)
            name = joint.get("name")
            frame = transform(joint.find("origin"))
            if name == "camera_mount" and side == "left":
                # Rotate the complete left camera/bracket to the outer wrist side.
                flip = np.eye(4)
                flip[:3, :3] = Rotation.from_euler("z", math.pi).as_matrix()
                frame = flip @ frame
            set_origin(joint, frame)
            joint.set("name", prefix + name)
            for tag in ("parent", "child"):
                node = joint.find(tag)
                node.set("link", prefix + node.get("link"))
            mimic = joint.find("mimic")
            if mimic is not None:
                mimic.set("joint", prefix + mimic.get("joint"))
            root.append(joint)
        side_linkage = deepcopy(linkage)
        for attribute in ("name", "driver", "sliders", "rods"):
            side_linkage.set(
                attribute,
                " ".join(prefix + name for name in linkage.get(attribute).split()),
            )
        root.append(side_linkage)
        mount = ET.SubElement(
            root, "joint", name=prefix + "gripper_mount", type="fixed"
        )
        ET.SubElement(mount, "parent", link=prefix + "wrist_roll_link")
        ET.SubElement(mount, "child", link=prefix + "gripper_base")
        ET.SubElement(
            mount,
            "origin",
            xyz=f"0 0 {ATTACHMENT_Z + HOUSING_REAR_Z}",
            rpy=f"{math.pi} 0 0",
        )
        ET.SubElement(root, "link", name=prefix + "gripper_tcp")
        tcp = ET.SubElement(
            root, "joint", name=prefix + "gripper_tcp_mount", type="fixed"
        )
        ET.SubElement(tcp, "parent", link=prefix + "gripper_base")
        ET.SubElement(tcp, "child", link=prefix + "gripper_tcp")
        ET.SubElement(tcp, "origin", xyz=f"0 0 {TCP_Z}", rpy=f"{math.pi} 0 0")
    for material in source.findall("material"):
        if root.find(f"material[@name='{material.get('name')}']") is None:
            root.insert(0, deepcopy(material))
    anonymous = root.find("material[@name='anonymous_end_effector']")
    if anonymous is not None:
        root.remove(anonymous)
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(URDF, encoding="utf-8", xml_declaration=True)


if __name__ == "__main__":
    main()
