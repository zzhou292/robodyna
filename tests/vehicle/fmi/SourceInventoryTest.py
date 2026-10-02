"""Keep original Vehicle FMI source, artifacts and real main programs distinct."""
import ast
import hashlib
import json
from pathlib import Path
import shlex
import sys
import unittest
import uuid

from tools.verification.cmake_evidence import commands

EXPORTS = Path(sys.argv.pop(1))
BASELINE = Path(sys.argv.pop(1))
LIB_BUILD = Path(sys.argv.pop(1))
DEMO_BUILD = Path(sys.argv.pop(1))
ANCHOR = Path(sys.argv.pop(1))
ROOT = ANCHOR.parents[3]


def calls(path, name):
    return [node.value for node in ast.parse(path.read_text()).body
            if isinstance(node, ast.Expr) and isinstance(node.value, ast.Call)
            and isinstance(node.value.func, ast.Name) and node.value.func.id == name]


class VehicleFmiSources(unittest.TestCase):
    def test_all_retained_sources_and_resources_are_unchanged(self):
        rows = json.loads(BASELINE.read_text())["files"]
        self.assertEqual(len(rows), 57)
        for row in rows:
            with self.subTest(path=row["path"]):
                self.assertEqual(hashlib.sha256((ROOT / row["path"]).read_bytes()).hexdigest(), row["sha256"])

    def test_six_exporters_match_original_cmake_and_uuid_policy(self):
        manifest = json.loads(EXPORTS.read_text())
        self.assertEqual(len(manifest["exports"]), 6)
        self.assertEqual(len(calls(LIB_BUILD, "native_fmu")), 6)
        for row in manifest["exports"]:
            values = {}
            for command in commands((ROOT / row["cmake"]).read_text()):
                words = shlex.split(command["arguments"])
                if command["command"] == "set" and len(words) >= 2:
                    values[words[0]] = words[1]
            self.assertEqual(values["COMPONENT_NAME"], row["identifier"])
            self.assertEqual(values["COMPONENT_RESOURCES_DIR"], "${CMAKE_SOURCE_DIR}/" + row["resource_directory"])
            self.assertEqual(row["guid"], str(uuid.uuid5(uuid.UUID(manifest["guid_namespace"]), row["identifier"])))

    def test_five_original_mains_compile_as_executables(self):
        binaries = calls(DEMO_BUILD, "cc_binary")
        self.assertEqual(len(binaries), 5)
        sources = []
        for call in binaries:
            srcs = next(k.value for k in call.keywords if k.arg == "srcs")
            sources.extend(node.value for node in ast.walk(srcs) if isinstance(node, ast.Constant))
        self.assertEqual(len(set(sources)), 5)
        for source in sources:
            self.assertIn("int main(", (ROOT / "src/demos/vehicle/fmi" / source).read_text())


if __name__ == "__main__":
    unittest.main()
