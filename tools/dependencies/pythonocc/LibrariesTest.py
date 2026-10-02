import unittest
from tools.dependencies.pythonocc.libraries import dependency_order


class NativeLibraryClosureTest(unittest.TestCase):
    def test_shared_dependencies_are_loaded_once_without_unneeded_libraries(self):
        libraries = {"libTKBody": {"needed": ["libTKMath", "libTKernel"]},
                     "libTKMath": {"needed": ["libTKernel", "libc.so.6"]},
                     "libTKernel": {"needed": []}, "libTKUnused": {"needed": []}}
        self.assertEqual(dependency_order(libraries, ["libTKBody", "libTKMath"]),
                         ["libTKernel", "libTKMath", "libTKBody"])

    def test_missing_and_cyclic_required_libraries_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "not admitted"):
            dependency_order({"libTKA": {"needed": ["libTKMissing"]}}, ["libTKA"])
        with self.assertRaisesRegex(ValueError, "dependency cycle"):
            dependency_order({"libTKA": {"needed": ["libTKB"]}, "libTKB": {"needed": ["libTKA"]}}, ["libTKA"])


if __name__ == "__main__":
    unittest.main()
