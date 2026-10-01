import unittest
from tools.bindings.run_swig import tool_environment


class NestedToolEnvironmentTests(unittest.TestCase):
    def test_child_owns_runfiles_without_losing_workstation_guards(self):
        parent = {"RUNFILES_DIR": "parent", "RUNFILES_MANIFEST_FILE": "parent-manifest",
                  "PYTHON_RUNFILES": "parent-python", "PYTHONPATH": "parent-imports",
                  "PYTHONHOME": "parent-runtime", "PATH": "/usr/bin",
                  "OMP_NUM_THREADS": "1", "CUDA_VISIBLE_DEVICES": ""}
        child = tool_environment(parent)
        self.assertEqual(child, {"PATH": "/usr/bin", "OMP_NUM_THREADS": "1", "CUDA_VISIBLE_DEVICES": ""})
        self.assertEqual(parent["RUNFILES_DIR"], "parent")


if __name__ == "__main__":
    unittest.main()
