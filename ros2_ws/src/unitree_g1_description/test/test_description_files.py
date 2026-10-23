import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).parents[1]


class DescriptionFileTests(unittest.TestCase):
    def test_urdf_and_package_xml_parse(self) -> None:
        for relative_path in ["package.xml", "urdf/g1_29dof_mode_15_brainco_hand.urdf", "urdf/g1_29dof_dex3.urdf"]:
            ET.parse(PACKAGE_ROOT / relative_path)

    def test_joint_config_contains_nonempty_joint_map(self) -> None:
        path = PACKAGE_ROOT / "config" / "g1_29dof_dex3_joints.yaml"
        data = yaml.safe_load(path.read_text())

        self.assertIsInstance(data, dict)
        self.assertTrue(data)
        self.assertEqual(len(data['dex3_hands']['joints']),7)
        self.assertEqual(data['dex3_hands']['right_state_topic'],'/lf/dex3/right/state')


if __name__ == "__main__":
    unittest.main()
