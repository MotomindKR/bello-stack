import unittest
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

from mini_torso_grippers import URDF, transform


def visual_mesh(link, name=None):
    visual = link.find("visual" if name is None else f"visual[@name='{name}']")
    mesh = visual.find("geometry/mesh")
    result = trimesh.load_scene(URDF.parent / mesh.get("filename")).to_mesh()
    result.apply_scale(np.fromstring(mesh.get("scale", "1 1 1"), sep=" "))
    result.apply_transform(transform(visual.find("origin")))
    return result


class GripperMountTests(unittest.TestCase):
    def test_camera_bracket_seats_flush_and_housing_clears_both_wrists(self):
        root = ET.parse(URDF).getroot()
        for side in ("left", "right"):
            with self.subTest(side=side):
                wrist = visual_mesh(root.find(f"link[@name='{side}_wrist_roll_link']"))
                housing = visual_mesh(
                    root.find(f"link[@name='{side}_gripper_base']"),
                    f"{side}_gripper_body",
                )
                mount = root.find(f"joint[@name='{side}_gripper_mount']")
                housing.apply_transform(transform(mount.find("origin")))
                camera = visual_mesh(root.find(f"link[@name='{side}_camera_link']"))
                camera.apply_transform(
                    transform(root.find(f"joint[@name='{side}_camera_mount']/origin"))
                )
                camera.apply_transform(transform(mount.find("origin")))
                gap = wrist.bounds[0, 2] - camera.bounds[1, 2]
                self.assertAlmostEqual(gap, 0.0, delta=1e-9)
                self.assertAlmostEqual(
                    wrist.bounds[0, 2] - housing.bounds[1, 2], 0.012, delta=1e-6
                )
                np.testing.assert_allclose(transform(mount.find("origin"))[:2, 3], 0.0)
                self.assertLess(transform(mount.find("origin"))[2, 2], -0.999999)


if __name__ == "__main__":
    unittest.main()
