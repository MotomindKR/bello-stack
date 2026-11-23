#!/usr/bin/env python3
"""Apply Milly's collision primitives or preview them offline.

python scripts/milly_collisions.py
uv run --with numpy --with scipy --with trimesh --with fast-simplification \
    --with plotly scripts/milly_collisions.py --preview /tmp/milly-collisions.html
"""

from __future__ import annotations

import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

NS = "https://motomind.kr/bello/urdf"
ET.register_namespace("bello", NS)


def numbers(values):
    return " ".join(f"{v:.12g}" for v in values)


def apply_collisions(root, description):
    spec = json.loads((description / "collision-primitives.json").read_text())
    if spec["units"] != "m":
        raise ValueError("Collision dimensions must be in metres")
    links = {link.get("name"): link for link in root.findall("link")}
    replacements = {name: [] for name in links}
    names = set()
    for shape in spec["shapes"]:
        name = shape["link"] + "_" + shape["name"]
        if name in names or shape["link"] not in links:
            raise ValueError(f"Duplicate collision or unknown link: {name}")
        names.add(name)
        for key in ("xyz", "rpy"):
            if len(shape[key]) != 3 or not all(math.isfinite(v) for v in shape[key]):
                raise ValueError(f"Invalid {key}: {name}")
        node = ET.Element("collision", name=name)
        ET.SubElement(
            node, "origin", xyz=numbers(shape["xyz"]), rpy=numbers(shape["rpy"])
        )
        geometry = ET.SubElement(node, "geometry")
        if shape["type"] == "box":
            size = shape["size"]
            if len(size) != 3 or not all(math.isfinite(v) and v > 0 for v in size):
                raise ValueError(f"Invalid box: {name}")
            ET.SubElement(geometry, "box", size=numbers(size))
        elif shape["type"] == "cylinder":
            if not all(
                math.isfinite(shape[k]) and shape[k] > 0 for k in ("radius", "length")
            ):
                raise ValueError(f"Invalid cylinder: {name}")
            ET.SubElement(
                geometry,
                "cylinder",
                radius=str(shape["radius"]),
                length=str(shape["length"]),
            )
        else:
            raise ValueError(f"Unsupported collision primitive: {shape['type']}")
        replacements[shape["link"]].append(node)
    for name, link in links.items():
        for node in link.findall("collision"):
            link.remove(node)
        link.extend(replacements[name])
    return len(names)


def preview(root, description, output):
    import numpy as np
    import plotly.graph_objects as go
    import trimesh
    from plotly.subplots import make_subplots
    from scipy.spatial.transform import Rotation

    def transform(node):
        matrix = np.eye(4)
        if node is not None:
            matrix[:3, 3] = np.fromstring(node.get("xyz", "0 0 0"), sep=" ")
            matrix[:3, :3] = Rotation.from_euler(
                "xyz", np.fromstring(node.get("rpy", "0 0 0"), sep=" ")
            ).as_matrix()
        return matrix

    linkage = root.find(f"{{{NS}}}slider_crank")

    def poses(q):
        positions = {linkage.attrib["driver"]: q}
        r, length, h, phase, reference = (
            float(linkage.get(k))
            for k in (
                "crank_radius",
                "rod_length",
                "slider_offset",
                "phase",
                "slider_reference",
            )
        )
        theta = q + phase
        v = h - r * math.sin(theta)
        d = math.sqrt(length * length - v * v)
        positions.update(
            {
                j: r * math.cos(theta) + d - reference
                for j in linkage.get("sliders").split()
            }
        )
        positions.update(
            {j: math.atan2(v, d) - theta for j in linkage.get("rods").split()}
        )
        frames = {"base_link": np.eye(4)}
        pending = list(root.findall("joint"))
        while pending:
            ready = [j for j in pending if j.find("parent").get("link") in frames]
            if not ready:
                raise ValueError("Disconnected URDF tree")
            for joint in ready:
                motion = np.eye(4)
                if joint.get("type") != "fixed":
                    axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
                    qj = positions.get(joint.attrib["name"], 0.0)
                    if joint.get("type") == "prismatic":
                        motion[:3, 3] = axis * qj
                    else:
                        motion[:3, :3] = Rotation.from_rotvec(axis * qj).as_matrix()
                frames[joint.find("child").get("link")] = (
                    frames[joint.find("parent").get("link")]
                    @ transform(joint.find("origin"))
                    @ motion
                )
                pending.remove(joint)
        return frames

    colors = [
        "#718096",
        "#db718b",
        "#4c91de",
        "#56b89b",
        "#ab83d8",
        "#e5ab52",
        "#429ea6",
        "#ed926d",
        "#7395e1",
        "#c081d0",
        "#86af65",
        "#cba254",
        "#e0769a",
    ]
    meshes = []
    for index, link in enumerate(
        l for l in root.findall("link") if l.findall("collision") or l.findall("visual")
    ):
        name = link.get("name")
        for kind in ("collision", "visual"):
            parts = []
            for node in link.findall(kind):
                geom = node.find("geometry")
                if kind == "visual":
                    mesh = trimesh.load_mesh(
                        description / "urdf" / geom.find("mesh").get("filename")
                    )
                    if len(mesh.faces) > 1800:
                        mesh = mesh.simplify_quadric_decimation(face_count=1800)
                elif geom.find("box") is not None:
                    mesh = trimesh.creation.box(
                        np.fromstring(geom.find("box").get("size"), sep=" ")
                    )
                else:
                    c = geom.find("cylinder")
                    mesh = trimesh.creation.cylinder(
                        float(c.get("radius")), float(c.get("length")), sections=32
                    )
                mesh.apply_transform(transform(node.find("origin")))
                parts.append(mesh)
            if not parts:
                continue
            meshes.append(
                (
                    name,
                    kind,
                    trimesh.util.concatenate(parts),
                    colors[index % len(colors)],
                )
            )

    figure = make_subplots(
        rows=1,
        cols=2,
        specs=[[{"type": "scene"}, {"type": "scene"}]],
        subplot_titles=(
            "Whole arm · zero arm joint angles",
            "Gripper and camera · detail",
        ),
    )
    children = {}
    for joint in root.findall("joint"):
        parent = joint.find("parent").attrib["link"]
        children.setdefault(parent, []).append(joint.find("child").attrib["link"])
    detail_links = {"gripper_base", "camera_link"}
    pending = list(detail_links)
    while pending:
        parent = pending.pop()
        for child in children.get(parent, ()):
            if child not in detail_links:
                detail_links.add(child)
                pending.append(child)
    records = []
    for col in (1, 2):
        for name, kind, mesh, color in meshes:
            if col == 2 and name not in detail_links:
                continue
            records.append((name, kind, mesh, col))
            figure.add_trace(
                go.Mesh3d(
                    x=[],
                    y=[],
                    z=[],
                    i=mesh.faces[:, 0],
                    j=mesh.faces[:, 1],
                    k=mesh.faces[:, 2],
                    color=color if kind == "collision" else "#9099a7",
                    opacity=1 if kind == "collision" else 0.18,
                    visible=kind == "collision",
                    name=name + " · " + kind,
                    legendgroup=name,
                    showlegend=col == 1 and kind == "collision",
                    flatshading=True,
                    hovertemplate=name + " · " + kind + "<extra></extra>",
                    lighting={"ambient": 0.65, "diffuse": 0.8, "specular": 0.15},
                ),
                row=1,
                col=col,
            )

    def vertices(q):
        frames = poses(q)
        points = []
        for name, _, mesh, col in records:
            frame = (
                frames[name]
                if col == 1
                else np.linalg.inv(frames["gripper_base"]) @ frames[name]
            )
            v = trimesh.transform_points(mesh.vertices, frame)
            points.append({"x": v[:, 0], "y": v[:, 1], "z": v[:, 2]})
        return points

    for trace, points in zip(figure.data, vertices(-2.2)):
        trace.update(points)
    frames = []
    for q, label in [(-2.2, "Open"), (-1.1, "Half"), (0.0, "Closed")]:
        frames.append(
            go.Frame(
                name=label,
                data=[go.Mesh3d(**p) for p in vertices(q)],
                traces=list(range(len(records))),
            )
        )
    figure.frames = frames
    home = poses(-2.2)
    for q in (-1.1, 0.0):
        assert np.allclose(poses(q)["camera_link"], home["camera_link"])
    assert np.allclose(home["camera_link"][:3, 0], home["camera_optical_frame"][:3, 2])
    for col in (1, 2):
        camera = home["camera_link"]
        if col == 2:
            camera = np.linalg.inv(home["gripper_base"]) @ camera
        start = camera[:3, 3]
        for axis, color, length, label in [
            (0, "#e53935", 0.075, "Camera +X / optical +Z · view"),
            (1, "#20994b", 0.035, "Camera +Y"),
            (2, "#246ee9", 0.035, "Camera +Z"),
        ]:
            direction = camera[:3, axis]
            end = start + direction * length
            figure.add_trace(
                go.Scatter3d(
                    x=[start[0], end[0]],
                    y=[start[1], end[1]],
                    z=[start[2], end[2]],
                    mode="lines+text",
                    line={"color": color, "width": 6},
                    text=[
                        "",
                        "+X / optical +Z (view)" if axis == 0 else f"+{'XYZ'[axis]}",
                    ],
                    textposition="top center",
                    textfont={"color": color, "size": 12},
                    name=label,
                    showlegend=col == 1,
                    hovertemplate=label + "<extra></extra>",
                ),
                row=1,
                col=col,
            )
            figure.add_trace(
                go.Cone(
                    x=[end[0]],
                    y=[end[1]],
                    z=[end[2]],
                    u=[direction[0]],
                    v=[direction[1]],
                    w=[direction[2]],
                    anchor="tip",
                    sizemode="absolute",
                    sizeref=0.008,
                    colorscale=[[0, color], [1, color]],
                    showscale=False,
                    showlegend=False,
                    hoverinfo="skip",
                ),
                row=1,
                col=col,
            )
    visibility = lambda mode: (
        [
            mode == "Overlay"
            or kind == ("visual" if mode == "Exterior" else "collision")
            for _, kind, _, _ in records
        ]
        + [True] * (len(figure.data) - len(records))
    )
    figure.update_layout(
        title={
            "text": f"Milly collision shapes · {len(root.findall('.//collision'))} primitives · boxes and cylinders",
            "x": 0.02,
            "y": 0.98,
        },
        paper_bgcolor="#f5f7fa",
        margin={"l": 15, "r": 15, "t": 190, "b": 80},
        height=900,
        legend={"orientation": "h", "y": -0.07},
        updatemenus=[
            {
                "type": "buttons",
                "direction": "right",
                "x": 0,
                "y": 1.16,
                "buttons": [
                    {
                        "label": mode,
                        "method": "restyle",
                        "args": [{"visible": visibility(mode)}],
                    }
                    for mode in ("Collision shapes", "Overlay", "Exterior")
                ],
            }
        ],
        sliders=[
            {
                "active": 0,
                "x": 0.55,
                "len": 0.4,
                "y": 1.26,
                "currentvalue": {"prefix": "Gripper: "},
                "steps": [
                    {
                        "label": f.name,
                        "method": "animate",
                        "args": [
                            [f.name],
                            {
                                "mode": "immediate",
                                "frame": {"duration": 0, "redraw": True},
                                "transition": {"duration": 0},
                            },
                        ],
                    }
                    for f in frames
                ],
            }
        ],
    )
    scene = {
        "aspectmode": "data",
        "xaxis": {"title": "X (m)"},
        "yaxis": {"title": "Y (m)"},
        "zaxis": {"title": "Z (m)"},
        "bgcolor": "#f5f7fa",
        "camera": {"eye": {"x": -1.6, "y": 2, "z": 1.2}},
        "uirevision": "orbit",
    }
    figure.update_layout(
        scene=scene, scene2={**scene, "camera": {"eye": {"x": 1.5, "y": -2, "z": 1.5}}}
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.write_html(
        output,
        include_plotlyjs=True,
        auto_open=False,
        auto_play=False,
        config={"displaylogo": False, "responsive": True},
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--description",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "descriptions/milly",
    )
    parser.add_argument("--preview", type=Path)
    args = parser.parse_args()
    path = args.description / "urdf/milly.urdf"
    root = ET.parse(path).getroot()
    count = apply_collisions(root, args.description)
    ET.indent(root, space="  ")
    path.write_bytes(ET.tostring(root, encoding="utf-8", xml_declaration=True) + b"\n")
    print(f"Wrote {count} primitive colliders to {path}")
    if args.preview:
        preview(root, args.description, args.preview)
        print(args.preview)


if __name__ == "__main__":
    main()
