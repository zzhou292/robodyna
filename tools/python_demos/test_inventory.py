"""Source audit checks distinguish actual imports from comments and helpers."""

import json
from pathlib import Path
import tempfile
import unittest

from tools.python_demos.inventory import inspect_file, module_leaf


class PythonDemoInventory(unittest.TestCase):
    def test_imports_and_helpers_are_read_without_executing_top_level_code(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            main = root / "demo.py"
            main.write_text("import pychrono.core as chrono\nimport numpy as np\nfrom helper import Model\n"
                            "# import tensorflow\nraise RuntimeError('must not execute')\n"
                            "x = chrono.GetChronoDataFile('models/mesh.obj')\n"
                            "body = chrono.ChBody()\n")
            (root / "helper.py").write_text("import pychrono.fea as fea\n")
            result = inspect_file(main, root)
            self.assertEqual(result["native_modules"], ["core"])
            self.assertEqual(result["external_packages"], ["numpy"])
            self.assertEqual(result["helpers"], ["helper.py"])
            self.assertIn({"module": "core", "name": "ChBody"}, result["potential_native_references"])
            self.assertEqual(result["literal_or_dynamic_asset_requests"][0]["literal"], "models/mesh.obj")

    def test_dynamic_assets_do_not_claim_a_closed_runtime_bundle(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            main = root / "demo.py"
            main.write_text("import pychrono as chrono\npath = chrono.GetChronoDataFile(user_choice)\n")
            result = inspect_file(main, root)
            self.assertTrue(result["literal_or_dynamic_asset_requests"][0]["dynamic"])
            self.assertIsNone(result["literal_or_dynamic_asset_requests"][0]["literal"])
            self.assertEqual(module_leaf("pychrono"), module_leaf("pychrono.core"))


if __name__ == "__main__":
    unittest.main()
