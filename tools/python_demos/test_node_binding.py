"""Check declared-node routing only; these mocks do not qualify ROS physics/IPC."""

import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

from tools.python_demos import execute


class DeclaredRosNodeBinding(unittest.TestCase):
    def test_create_only_work_directory_binds_exact_node_before_script(self):
        previous = Path.cwd()
        old_path = list(sys.path)
        try:
            with tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                node = root / "declared-node"
                node.write_text("not executed by this control test")
                script = root / "script.py"
                script.write_text("from pathlib import Path\nassert Path('chrono_ros_node').is_symlink()\n")
                package = root / "package.json"
                package.write_text("{}")
                output = root / "run"
                args = ["executor", "--script", str(script), "--package-manifest", str(package),
                        "--case", "node-control", "--data-root", str(root),
                        "--output-dir", str(output), "--ros-node", str(node)]
                product = mock.Mock(vehicle=None)
                with mock.patch.object(execute.sys, "argv", args), \
                        mock.patch.object(execute.importlib, "import_module", return_value=product):
                    execute.main()
                self.assertTrue((output / "work/chrono_ros_node").samefile(node))
                result = json.loads((output / "result.json").read_text())
                self.assertEqual(result["exit_code"], 0)
                self.assertTrue(result["script_started"])
                with mock.patch.object(execute.sys, "argv", args):
                    with self.assertRaises(FileExistsError):
                        execute.main()
                self.assertTrue((output / "work/chrono_ros_node").samefile(node))
        finally:
            os.chdir(previous)
            sys.path[:] = old_path

    def test_missing_declared_node_records_failure_before_import_or_script(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            script = root / "script.py"
            script.write_text("raise AssertionError('must not start')")
            package = root / "package.json"
            package.write_text("{}")
            output = root / "run"
            args = ["executor", "--script", str(script), "--package-manifest", str(package),
                    "--case", "node-control", "--data-root", str(root),
                    "--output-dir", str(output), "--ros-node", str(root / "missing")]
            with mock.patch.object(execute.sys, "argv", args), \
                    mock.patch.object(execute.importlib, "import_module") as imported:
                with self.assertRaisesRegex(ValueError, "node launcher is missing"):
                    execute.main()
            imported.assert_not_called()
            result = json.loads((output / "result.json").read_text())
            self.assertEqual(result["exit_code"], 1)
            self.assertFalse(result["script_started"])


if __name__ == "__main__":
    unittest.main()
