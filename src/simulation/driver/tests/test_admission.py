import json
from pathlib import Path
import tempfile
import unittest
import stat
import zipfile

from src.simulation.driver.jsonio import read_json
from src.simulation.driver.manifests import load_case, load_resources
from src.simulation.driver.sources import verify_sources, extract_members
from .fixture import case_fixture, write


class AdmissionTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.case, self.resources = case_fixture(self.root)

    def test_manifest_relative_sources_are_explicit_and_no_physics_is_claimed(self):
        case = load_case(self.root / "case.json")
        receipt = verify_sources(case)
        self.assertFalse(receipt["physics_prepared"])
        self.assertFalse(receipt["gpu_used"])
        self.assertFalse(receipt["canonical_arrays_authenticated"])
        self.assertEqual(load_resources(self.root / "resources.json")["workstation_lock"],
                         str(self.root / "workstation.lock"))

    def test_duplicate_unknown_and_nonfinite_json_rejected(self):
        for text in ('{"field":1,"field":2}', '{"field":NaN}'):
            (self.root / "bad.json").write_text(text)
            with self.assertRaises(ValueError):
                read_json(self.root / "bad.json")
        self.case["unexpected"] = 1
        write(self.root / "case.json", self.case)
        with self.assertRaisesRegex(ValueError, "unknown fields"):
            load_case(self.root / "case.json")

    def test_boolean_count_and_unknown_contact_profile_rejected(self):
        for key, value in (("samples", True), ("contact_activity", "silent_fallback")):
            changed = json.loads(json.dumps(self.case))
            changed["run"][key] = value
            write(self.root / "case.json", changed)
            with self.assertRaises(ValueError):
                load_case(self.root / "case.json")

    def test_horizon_and_resource_limits_rejected(self):
        self.case["run"]["duration_s"] = .2
        write(self.root / "case.json", self.case)
        with self.assertRaises(ValueError):
            load_case(self.root / "case.json")
        self.resources["cooperative_maximum_elapsed_s"] = self.resources["timeout_s"]
        write(self.root / "resources.json", self.resources)
        with self.assertRaisesRegex(ValueError, "cooperative"):
            load_resources(self.root / "resources.json")

    def test_changed_source_pin_rejected_before_extraction(self):
        case = load_case(self.root / "case.json")
        (self.root / "scope.json").write_bytes(b"changed")
        with self.assertRaisesRegex(ValueError, "identity changed"):
            verify_sources(case)
        self.assertFalse((self.root / "sources").exists())

    def test_wrong_member_hash_rejected_and_partial_extract_not_reused(self):
        self.case["original_members"]["member"]["sha256"] = "0" * 64
        write(self.root / "case.json", self.case)
        case = load_case(self.root / "case.json")
        with self.assertRaisesRegex(ValueError, "member identity"):
            extract_members(case, self.root / "sources")
        with self.assertRaises(FileExistsError):
            extract_members(case, self.root / "sources")

    def test_member_output_preserves_original_basename_without_directories(self):
        name = "original/authenticated-vehicle.key"
        with zipfile.ZipFile(self.root / "source.zip", "a") as archive:
            archive.writestr(name, b"member\n")
        self.case["original_members"]["member"]["member"] = name
        write(self.root / "case.json", self.case)
        case = load_case(self.root / "case.json")
        paths = extract_members(case, self.root / "sources")
        self.assertEqual(set(paths), set(self.case["original_members"]))
        self.assertEqual({Path(path).parent for path in paths.values()}, {self.root / "sources"})
        self.assertEqual(Path(paths["member"]).name, "authenticated-vehicle.key")
        self.assertEqual(Path(paths["member"]).read_bytes(), b"member\n")

    def test_duplicate_basenames_and_parent_traversal_are_rejected(self):
        for name in ("other/auxiliary_member.key", "../vehicle.key", "/absolute/vehicle.key"):
            changed = json.loads(json.dumps(self.case))
            changed["original_members"]["member"]["member"] = name
            write(self.root / "case.json", changed)
            with self.assertRaises(ValueError):
                load_case(self.root / "case.json")

    def test_zip_symbolic_link_is_rejected_without_following_it(self):
        name = "original/linked.key"
        with zipfile.ZipFile(self.root / "source.zip", "a") as archive:
            info = zipfile.ZipInfo(name)
            info.create_system = 3
            info.external_attr = (stat.S_IFLNK | 0o777) << 16
            archive.writestr(info, b"member\n")
        self.case["original_members"]["member"]["member"] = name
        write(self.root / "case.json", self.case)
        with self.assertRaisesRegex(ValueError, "symbolic link"):
            extract_members(load_case(self.root / "case.json"), self.root / "sources")


if __name__ == "__main__":
    unittest.main()
