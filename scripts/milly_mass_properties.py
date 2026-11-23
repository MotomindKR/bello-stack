#!/usr/bin/env python3
"""Apply pinned SDK mass properties in Milly's existing CAD-registered link frames.

uv run --with numpy --with scipy scripts/milly_mass_properties.py

The SDK wrist mass includes the moving linkage at the CAD reference pose.
"""

from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

ET.register_namespace("bello", "https://motomind.kr/bello/urdf")


def tensor(values):
    return np.array(
        [
            [
                values["i" + a + b] if "i" + a + b in values else values["i" + b + a]
                for b in "xyz"
            ]
            for a in "xyz"
        ]
    )


def parallel_axis(center):
    return np.eye(3) * np.dot(center, center) - np.outer(center, center)


def reframe(properties, frame):
    mass, center, inertia = properties
    rotation = frame[:3, :3]
    return mass, rotation @ center + frame[:3, 3], rotation @ inertia @ rotation.T


def validate(properties, name):
    mass, center, inertia = properties
    if not (
        np.isfinite(mass)
        and mass > 0
        and np.isfinite(center).all()
        and np.isfinite(inertia).all()
    ):
        raise ValueError(f"Invalid mass properties: {name}")
    if not np.allclose(inertia, inertia.T, atol=1e-14):
        raise ValueError(f"Asymmetric inertia: {name}")
    principal = np.linalg.eigvalsh(inertia)
    if principal[0] <= 0 or principal[0] + principal[1] < principal[2] - 1e-14:
        raise ValueError(f"Nonphysical principal inertia: {name}: {principal}")


def apply_mass_properties(root, description, cad_to_link=None):
    recipe = json.loads((description / "mass-properties.json").read_text())
    if cad_to_link is None:
        cad_to_link = json.loads((description / "exterior-assets.json").read_text())[
            "cad_to_link"
        ]
    frames = {name: np.asarray(frame) for name, frame in cad_to_link.items()}
    for name, frame in frames.items():
        r = frame[:3, :3]
        if not (
            np.allclose(r.T @ r, np.eye(3), atol=1e-10)
            and abs(np.linalg.det(r) - 1) < 1e-10
        ):
            raise ValueError(f"Invalid CAD registration rotation: {name}")
    properties = {}
    for name, source in recipe["links"].items():
        r = Rotation.from_euler("xyz", source["inertial_rpy_rad"]).as_matrix()
        original = (
            source["mass_kg"],
            np.asarray(source["center_m"]),
            r @ tensor(source["inertia_kg_m2"]) @ r.T,
        )
        properties[name] = reframe(
            original, frames[name] @ np.linalg.inv(source["source_link_from_cad"])
        )

    partition = recipe["linkage_partition"]
    aggregate = partition["aggregate_link"]
    mass, center, inertia = properties[aggregate]
    first_moment = mass * center
    about_origin = inertia + mass * parallel_axis(center)
    for part in partition["parts"]:
        name = part["link"]
        nominal = (
            part["mass_kg"],
            np.asarray(part["center_m"]),
            np.diag(part["diagonal_kg_m2"]),
        )
        properties[name] = nominal
        m, c, j = reframe(nominal, frames[aggregate] @ np.linalg.inv(frames[name]))
        mass -= m
        first_moment -= m * c
        about_origin -= j + m * parallel_axis(c)
    if mass <= 0:
        raise ValueError("Linkage partition exceeds SDK wrist mass")
    center = first_moment / mass
    properties[aggregate] = mass, center, about_origin - mass * parallel_axis(center)
    if (
        abs(sum(p[0] for p in properties.values()) - recipe["source_total_mass_kg"])
        > 1e-12
    ):
        raise ValueError("Mass partition changed the SDK total")

    links = {link.get("name"): link for link in root.findall("link")}
    for name, values in properties.items():
        validate(values, name)
        if name not in links:
            raise ValueError(f"Missing mass-property link: {name}")
    for name, (mass, center, inertia) in properties.items():
        link = links[name]
        for old in link.findall("inertial"):
            link.remove(old)
        inertial = ET.Element("inertial")
        ET.SubElement(
            inertial, "origin", xyz=" ".join(f"{x:.16g}" for x in center), rpy="0 0 0"
        )
        ET.SubElement(inertial, "mass", value=f"{mass:.16g}")
        ET.SubElement(
            inertial,
            "inertia",
            **{
                "i" + a + b: f"{inertia[i, j]:.16g}"
                for i, a in enumerate("xyz")
                for j, b in enumerate("xyz")
                if i <= j
            },
        )
        link.insert(0, inertial)
    actual_total = sum(
        float(m.get("value")) for m in root.findall("./link/inertial/mass")
    )
    if abs(actual_total - recipe["source_total_mass_kg"]) > 1e-12:
        raise ValueError("Unexpected additional mass in the URDF")
    return actual_total


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--description",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "descriptions/milly",
    )
    args = parser.parse_args()
    path = args.description / "urdf/milly.urdf"
    root = ET.parse(path).getroot()
    mass = apply_mass_properties(root, args.description)
    ET.indent(root, space="  ")
    path.write_bytes(ET.tostring(root, encoding="utf-8", xml_declaration=True) + b"\n")
    print(
        f"Applied SDK mass properties: {mass:.12f} kg; current joint/control definitions retained"
    )


if __name__ == "__main__":
    main()
