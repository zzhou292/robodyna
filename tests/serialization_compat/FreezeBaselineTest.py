"""Check strict source admission and opaque-byte preservation of the new profile."""

import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import freeze_baseline as module


class SystemBaselineFreeze(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        self.repo = root / "repo"
        self.generated = root / "generated"
        self.destination = root / "frozen"
        self.binary = root / "producer"
        self.generated.mkdir()
        self.binary.write_bytes(b"test producer bytes")
        self.head = "1" * 40
        self.committed = {}
        for name in module.SYSTEM_PRODUCTION_INPUTS:
            path = self.repo / name
            path.parent.mkdir(parents=True, exist_ok=True)
            data = f"// original source: {name}\n".encode()
            path.write_bytes(data)
            self.committed[name] = data
        for name in module.SYSTEM_PRODUCER_INPUTS:
            path = self.repo / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("// producer input\n")
        self.payloads = {
            "baseline.json": b'{"assembly": "ChAssembly_000042"}\n',
            "baseline.xml": b'<assembly>ChAssembly_000073</assembly>\n',
            "baseline.bin": b'\x00ChAssembly_000091\xff',
            "producer.json": json.dumps({"compiler": "unit-test compiler", "pointer_bytes": 8}).encode(),
        }
        for name, data in self.payloads.items():
            (self.generated / name).write_bytes(data)

    def git(self, arguments, cwd, text=False):
        self.assertEqual(Path(cwd), self.repo)
        if arguments == ["git", "rev-parse", "HEAD"]:
            self.assertTrue(text)
            return self.head + "\n"
        self.assertEqual(arguments[:2], ["git", "show"])
        self.assertTrue(arguments[2].startswith(self.head + ":"))
        return self.committed[arguments[2][41:]]

    def freeze(self):
        with patch.object(module.subprocess, "check_output", side_effect=self.git), contextlib.redirect_stdout(io.StringIO()):
            module.freeze(self.repo, self.generated, self.destination, self.binary, "system")

    def test_current_body_paths_and_unmodified_archive_bytes_are_recorded(self):
        self.freeze()
        manifest = json.loads((self.destination / "manifest.json").read_text())
        self.assertEqual(manifest["baseline_head"], self.head)
        self.assertEqual(manifest["profile"], "system")
        sources = manifest["production_inputs"]
        self.assertIn("src/mbd/bodies/RbBody.cpp", sources)
        self.assertIn("include/robodyna/mbd/RbBody.h", sources)
        self.assertIn(module.CHRONO + "physics/ChBodyFwd.h", sources)
        self.assertIn(module.CHRONO + "physics/ChSystem.cpp", sources)
        self.assertNotIn(module.CHRONO + "physics/ChBody.cpp", sources)
        for name, data in self.payloads.items():
            self.assertEqual((self.destination / name).read_bytes(), data)
            self.assertEqual(manifest["files"][name]["sha256"], module.digest(data))

    def test_changed_canonical_body_or_system_source_fails_before_publication(self):
        for name in ("src/mbd/bodies/RbBody.cpp", module.CHRONO + "physics/ChSystem.h"):
            with self.subTest(name=name):
                path = self.repo / name
                path.write_bytes(self.committed[name] + b"// uncommitted change\n")
                with self.assertRaisesRegex(ValueError, "changed from recorded HEAD"):
                    self.freeze()
                self.assertFalse(self.destination.exists())
                path.write_bytes(self.committed[name])

    def test_existing_evidence_is_not_overwritten(self):
        self.destination.mkdir()
        evidence = self.destination / "evidence"
        evidence.write_bytes(b"keep existing evidence")
        with patch.object(module.subprocess, "check_output") as git:
            with self.assertRaisesRegex(ValueError, "refusing overwrite"):
                module.freeze(self.repo, self.generated, self.destination, self.binary, "system")
            git.assert_not_called()
        self.assertEqual(evidence.read_bytes(), b"keep existing evidence")

    def test_earlier_profiles_keep_their_historical_body_location(self):
        for profile in ("actual-types", "rename-probe", "body"):
            with self.subTest(profile=profile):
                paths = module.production_paths(profile)
                self.assertIn(module.CHRONO + "physics/ChBody.cpp", paths)
                self.assertNotIn("src/mbd/bodies/RbBody.cpp", paths)


if __name__ == "__main__":
    unittest.main()
