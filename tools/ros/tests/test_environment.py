import json
from pathlib import Path
import tempfile
import unittest

from tools.ros.environment import ros_environment


class RosEnvironmentTest(unittest.TestCase):
    def test_baseline_paths_remain_exact_and_ambient_paths_are_replaced(self):
        result = ros_environment({"AMENT_PREFIX_PATH": "/unrelated", "LD_LIBRARY_PATH": "/unrelated",
                                  "RUNFILES_DIR": "/wrong", "ROS_LOCALHOST_ONLY": "1"}, "/sdk")
        self.assertEqual(result["AMENT_PREFIX_PATH"], "/sdk/opt/ros/humble")
        self.assertEqual(result["LD_LIBRARY_PATH"], "/sdk/lib:/sdk/opt/ros/humble/lib:/sdk/usr/lib/x86_64-linux-gnu")
        self.assertNotIn("RUNFILES_DIR", result)
        self.assertEqual(result["ROS_LOCALHOST_ONLY"], "1")

    def test_only_declared_custom_prefix_enters_both_search_paths(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            marker = root / "share/ament_index/resource_index/packages/chrono_ros_interfaces"
            marker.parent.mkdir(parents=True)
            marker.write_bytes(b"")
            receipt = root / "sdk.json"
            record = dict(schema="robodyna.ros_interfaces_sdk.v1", package="chrono_ros_interfaces", prefix=".")
            receipt.write_text(json.dumps(record))
            result = ros_environment({}, "/sdk", [receipt])
            self.assertEqual(result["AMENT_PREFIX_PATH"], str(root) + ":/sdk/opt/ros/humble")
            self.assertTrue(result["LD_LIBRARY_PATH"].startswith(str(root / "lib") + ":"))
            with self.assertRaisesRegex(ValueError, "duplicate"):
                ros_environment({}, "/sdk", [receipt, receipt])
            record["prefix"] = "../escape"
            receipt.write_text(json.dumps(record))
            with self.assertRaisesRegex(ValueError, "Unsupported"):
                ros_environment({}, "/sdk", [receipt])
            record["prefix"] = "."
            receipt.write_text(json.dumps(record))
            marker.unlink()
            with self.assertRaisesRegex(ValueError, "ament"):
                ros_environment({}, "/sdk", [receipt])


if __name__ == "__main__":
    unittest.main()
