"""Check the native translation units against the inherited CMake declarations.

This deliberately interprets only the source-declaration subset of the pinned
file. Unknown conditions fail closed; it is not a general CMake interpreter.
"""

import ast
import hashlib
import json
from pathlib import Path
import posixpath
import re
import sys
import unittest

from tools.migration.source_transform import index_entries, original_bytes

def read_manifest(path):
    assignments = {}
    for node in ast.parse(path.read_text()).body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1:
            assignments[node.targets[0].id] = ast.literal_eval(node.value)
    return assignments


def selected_cmake_sources(text):
    text = text.split("add_library(Chrono_core", 1)[0]
    text = "\n".join(line.split("#", 1)[0] for line in text.splitlines())
    flags = {
        "CH_ENABLE_MODULE_FEA": True,
        "CH_ENABLE_MODULE_FEA_MULTIPHYSICS": False,
        "CHRONO_THRUST_FOUND": False,
        "EMSCRIPTEN": False,
        "BUILD_BENCHMARKING": False,
        "HDF5_FOUND": False,
        "CHRONO_YAML_ENABLED": False,
    }
    values = {}
    stack = []
    active = True
    commands = re.findall(r"(?ims)^[ \t]*(set|list|if|else|endif)\s*\((.*?)\)", text)
    for command, body in commands:
        tokens = [token.strip('"') for token in re.findall(r'"[^"]*"|[^\s]+', body)]
        command = command.lower()
        if command == "if":
            if len(tokens) not in (1, 2) or (len(tokens) == 2 and tokens[0] != "NOT"):
                raise ValueError(f"Unreviewed CMake condition: {body}")
            condition = flags[tokens[-1]] != (tokens[0] == "NOT")
            stack.append((active, condition))
            active = active and condition
        elif command == "else":
            parent, condition = stack[-1]
            active = parent and not condition
        elif command == "endif":
            active = stack.pop()[0]
        elif active:
            name = tokens[0] if command == "set" else tokens[1]
            expressions = tokens[1:] if command == "set" else tokens[2:]
            expanded = []
            for token in expressions:
                if token.startswith("${") and token.endswith("}") and token.count("${") == 1:
                    expanded.extend(values.get(token[2:-1], []))
                elif token:
                    expanded.append(token)
            if command == "set":
                values[name] = expanded
            else:
                if tokens[0] != "APPEND":
                    raise ValueError(f"Unreviewed CMake list operation: {body}")
                values.setdefault(name, []).extend(expanded)
    if stack:
        raise ValueError("Unclosed CMake source-declaration condition")
    return {
        ("@workspace/" + path.removeprefix("${ROBODYNA_SOURCE_ROOT}/")
         if path.startswith("${ROBODYNA_SOURCE_ROOT}/")
         else posixpath.normpath("src/chrono/" + path))
        for path in values["Chrono_FILES"]
        if path.endswith((".cpp", ".cc", ".c"))
    }


class NativeSourceInventory(unittest.TestCase):
    def test_matches_reviewed_enabled_cmake_sources(self):
        manifest = read_manifest(MANIFEST)
        contents = CMAKE.read_bytes()
        entries = index_entries(json.loads(TRANSFORMATIONS.read_text()))
        entry = entries["src/compatibility/chrono/src/chrono/CMakeLists.txt"]
        self.assertEqual(entry["original_sha256"], manifest["CORE_CMAKE_SHA256"])
        original = original_bytes(TRANSFORMATIONS.parent.parent.parent, entry)
        self.assertEqual(hashlib.sha256(original).hexdigest(), manifest["CORE_CMAKE_SHA256"])
        paths = [path for group in manifest["NATIVE_SOURCE_GROUPS"].values() for path in group]
        self.assertEqual(len(paths), len(set(paths)), "A translation unit is listed more than once")
        self.assertEqual(set(paths), selected_cmake_sources(original.decode()))
        relocations = read_manifest(LOCATIONS)["SOURCE_RELOCATIONS"]
        self.assertTrue(set(relocations).issubset(paths), "Relocation does not identify an imported implementation")
        current = {"@workspace/" + relocations[path].removeprefix("//").replace(":", "/")
                   if path in relocations else path for path in paths}
        additions = read_manifest(VISUAL)["VISUAL_ADAPTER_SOURCES"]
        added_paths = {"@workspace/" + label.removeprefix("//").replace(":", "/") for label in additions}
        self.assertEqual(len(additions), len(added_paths), "An adapter is compiled more than once")
        self.assertFalse(current & added_paths, "Added adapter duplicates a retained source")
        current.update(added_paths)
        self.assertEqual(current, selected_cmake_sources(contents.decode()))


if __name__ == "__main__":
    MANIFEST = Path(sys.argv.pop(1))
    CMAKE = Path(sys.argv.pop(1))
    TRANSFORMATIONS = Path(sys.argv.pop(1))
    LOCATIONS = Path(sys.argv.pop(1))
    VISUAL = Path(sys.argv.pop(1))
    unittest.main()
