"""Filesystem and stdout-admission tests; these fixtures never execute physics."""

from pathlib import Path
import tempfile
import unittest

from tools.dependencies.cuda_math import file_hash
from tools.native_demos.cases import prepare_case, validate_build_system_output


class CasesTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.repo, self.data = self.root / "repository", self.root / "data-source"
        self.repo.mkdir()
        self.data.mkdir()
        (self.data / "retained.txt").write_text("unchanged")
        (self.repo / "main.cpp").write_text("int main() { return 0; }")
        self.binary = self.repo / "binary"
        self.binary.write_bytes(b"unexecuted admission fixture")
        self.preset = {"source": "main.cpp", "source_sha256": file_hash(self.repo / "main.cpp"), "binary": "binary",
                       "target": "//examples/test:fixture", "mode": "cpu_headless", "writes_into_data": False,
                       "arguments": [], "scope": "filesystem admission fixture only"}

    def tearDown(self):
        self.temporary.cleanup()

    def prepare(self, output):
        return prepare_case(self.repo, self.preset, self.binary, self.data, output)

    def test_working_directory_resolves_original_relative_data_convention(self):
        request = self.prepare(self.root / "case")
        work = Path(request["working_directory"])
        self.assertEqual((work / "../data/retained.txt").read_text(), "unchanged")
        self.assertTrue((work / "DEMO_OUTPUT").is_dir())
        self.assertEqual(request["data_root"], str(self.data))
        self.assertEqual((self.data / "retained.txt").read_text(), "unchanged")

    def test_existing_output_and_symlinked_parent_cannot_write_into_sources(self):
        destination = self.root / "case"
        self.prepare(destination)
        with self.assertRaisesRegex(ValueError, "create-only"):
            self.prepare(destination)
        alias = self.root / "repository-alias"
        alias.symlink_to(self.repo, target_is_directory=True)
        with self.assertRaisesRegex(ValueError, "outside source"):
            self.prepare(alias / "new-case")
        self.assertFalse((self.repo / "new-case").exists())

    def test_changed_or_non_headless_preset_is_not_admitted(self):
        self.preset["mode"] = "gpu"
        with self.assertRaisesRegex(ValueError, "headless CPU"):
            self.prepare(self.root / "case")
        self.preset["mode"] = "cpu_headless"
        (self.repo / "main.cpp").write_text("changed entrypoint")
        with self.assertRaisesRegex(ValueError, "entrypoint changed"):
            self.prepare(self.root / "case")

    def test_stdout_parser_rejects_frozen_or_nonfinite_simulation_reports(self):
        rows = [f"Time: {i*.05:.2f}  Steps: {i*5}  Slider X position: {(-1)**i}  Engine torque: 0.2" for i in range(1, 50)]
        value = validate_build_system_output("\n".join(rows))
        self.assertEqual(value["reported_solver_steps"], 245)
        self.assertEqual(value["slider_x_range"], 2)
        with self.assertRaisesRegex(ValueError, "non-finite"):
            validate_build_system_output("\n".join(rows).replace("Engine torque: 0.2", "Engine torque: nan", 1))
        with self.assertRaisesRegex(ValueError, "did not advance"):
            validate_build_system_output("\n".join(rows).replace("Steps: 10", "Steps: 5", 1))


if __name__ == "__main__":
    unittest.main()
