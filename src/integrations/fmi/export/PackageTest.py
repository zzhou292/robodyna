"""Packaging unit tests; the XML helper is mocked, not a physics qualification."""

import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest import mock
import zipfile

from src.integrations.fmi.export.package import package, relative_path, sha256, validate_description


class PackageTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.library = self.root / "test-input.bin"
        self.library.write_bytes(b"unit fixture, not an executable FMU\x00\xff")
        self.helper = self.root / "mock-helper"
        self.helper.write_bytes(b"unit test mock identity")
        self.resource = self.root / "texture.bin"
        self.resource.write_bytes(b"unchanged resource")
        self.spec = {
            "schema": "robodyna.fmu_build.v1", "identifier": "UnitFixture",
            "version": "2", "mode": "CoSimulation", "guid": "fixture-guid",
            "library": str(self.library), "helper": str(self.helper),
            "tree": str(self.root / "tree"), "archive": str(self.root / "test.fmu"),
            "description": str(self.root / "model.xml"), "receipt": str(self.root / "receipt.json"),
            "resources": [{"source": str(self.resource), "destination": "resources/texture.bin"}],
            "licenses": [],
        }

    def helper_success(self, command, **options):
        self.assertEqual(command[0], str(self.helper))
        self.assertEqual(command[2], "UnitFixture.so")
        self.assertEqual(Path(command[1]), self.root / "tree/binaries/linux64")
        (Path(command[3]) / "modelDescription.xml").write_text(
            '<fmiModelDescription fmiVersion="2.0" guid="fixture-guid">'
            '<CoSimulation modelIdentifier="UnitFixture"/><ModelVariables/>'
            '</fmiModelDescription>')
        return types.SimpleNamespace(returncode=0, stdout="mocked descriptor\n")

    def test_native_helper_boundary_and_packaged_bytes(self):
        original = sha256(self.library)
        with mock.patch("src.integrations.fmi.export.package.subprocess.run", side_effect=self.helper_success):
            package(self.spec)
        with zipfile.ZipFile(self.spec["archive"]) as archive:
            self.assertEqual(archive.read("binaries/linux64/UnitFixture.so"), self.library.read_bytes())
            self.assertEqual(archive.read("resources/texture.bin"), self.resource.read_bytes())
            self.assertIsNone(archive.testzip())
        receipt = json.loads(Path(self.spec["receipt"]).read_text())
        self.assertEqual(receipt["library_sha256"], original)
        self.assertEqual(sha256(self.library), original)
        self.assertIn("no trajectory", receipt["scope"])

    def test_declared_runtime_dso_is_copied_beside_the_native_model(self):
        runtime = self.root / "libDependency.so.1"
        runtime.write_bytes(b"unit fixture runtime library")
        self.spec["runtime_libraries"] = [{"source": str(runtime), "name": runtime.name}]
        with mock.patch("src.integrations.fmi.export.package.subprocess.run", side_effect=self.helper_success):
            package(self.spec)
        with zipfile.ZipFile(self.spec["archive"]) as archive:
            self.assertEqual(archive.read("binaries/linux64/libDependency.so.1"), runtime.read_bytes())
        receipt = json.loads(Path(self.spec["receipt"]).read_text())
        row = next(x for x in receipt["files"] if x["path"].endswith("libDependency.so.1"))
        self.assertEqual(row["sha256"], sha256(runtime))

    def test_runtime_library_cannot_escape_or_replace_the_native_model(self):
        for name in ("../outside.so", "nested/inside.so", "UnitFixture.so"):
            with self.subTest(name=name):
                self.spec["tree"] = str(self.root / ("invalid-tree-" + str(len(name))))
                self.spec["runtime_libraries"] = [{"source": str(self.library), "name": name}]
                with self.assertRaises(ValueError):
                    package(self.spec)
                self.assertFalse(Path(self.spec["archive"]).exists())

    def test_wrong_generated_identity_rejects(self):
        path = self.root / "wrong.xml"
        path.write_text('<fmiModelDescription fmiVersion="2.0" guid="other">'
                        '<CoSimulation modelIdentifier="UnitFixture"/><ModelVariables/>'
                        '</fmiModelDescription>')
        with self.assertRaisesRegex(ValueError, "identity differs"):
            validate_description(path, "UnitFixture", "2", "CoSimulation", "fixture-guid")

    def test_failed_native_generator_cannot_publish_an_archive(self):
        failed = types.SimpleNamespace(returncode=1, stdout="native failure")
        with mock.patch("src.integrations.fmi.export.package.subprocess.run", return_value=failed):
            with self.assertRaisesRegex(RuntimeError, "Native FMU description"):
                package(self.spec)
        self.assertFalse(Path(self.spec["archive"]).exists())
        self.assertFalse(Path(self.spec["receipt"]).exists())

    def test_paths_cannot_escape_the_fmu_tree(self):
        for name in ("/absolute", "../escape", "resources/../escape", "resources\\escape"):
            with self.subTest(name=name), self.assertRaises(ValueError):
                relative_path(name)


if __name__ == "__main__":
    unittest.main()
