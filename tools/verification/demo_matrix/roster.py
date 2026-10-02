"""Keep the full inherited source denominator independent of exposed targets."""

import collections
import json
from pathlib import Path

from tools.verification.chrono_inventory import digest, git, tree_files
from tools.verification.native_entrypoints import NATIVE_SUFFIXES, entrypoints


EXPECTED = {"native_demo": 311, "native_template": 1, "cuda_demo": 26, "python_demo": 97, "csharp_demo": 17}
INPUT_MANIFESTS = ["docs/migration/SOURCES.json", "examples/fea/cuda/demo_targets.json",
                   "examples/python/Inventory.json", "examples/python/Admission.json", "examples/csharp/DEMO_TARGETS.json"]


def build_metadata(repository):
    root = Path(repository)
    names = set(git(root, "ls-files", "--cached", "--others", "--exclude-standard", "-z").decode().split("\0"))
    selected = [name for name in names if name.endswith(("BUILD.bazel", ".bzl", ".cmake", ".sh", ".bash")) or
                Path(name).name in ("BUILD", "MODULE.bazel", "MODULE.bazel.lock", "CMakeLists.txt", ".bazelrc", ".bazelversion", ".bazelignore")]
    return {name: digest((root / name).read_bytes()) for name in sorted(selected) if (root / name).is_file()}


def read_roster(repository):
    root = Path(repository).resolve(strict=True)
    source_manifest = json.loads((root / INPUT_MANIFESTS[0]).read_text())
    pins = {r["component"]: r for r in source_manifest["sources"]}
    trees = {key: tree_files(root, pins[key]["source_tree"]) for key in ("chrono_capabilities", "fea")}
    result = []

    def add(family, relative, component="chrono_capabilities", declaration=None):
        pin = pins[component]
        if relative not in trees[component]:
            raise ValueError("Program is not in the preserved source tree: " + relative)
        source = pin["path"] + "/" + relative
        path = root / source
        if not path.is_file():
            raise ValueError("Preserved program is missing: " + source)
        result.append({"family": family, "source": source, "original_git_blob": trees[component][relative]["git_blob"],
                       "current_sha256": digest(path.read_bytes()), "declared_manifest": declaration})

    for name in sorted(trees["chrono_capabilities"]):
        if name.startswith("src/demos/") and Path(name).suffix in NATIVE_SUFFIXES:
            path = root / pins["chrono_capabilities"]["path"] / name
            if not path.is_file():
                raise ValueError("Native source missing: " + name)
            if entrypoints(path.read_text(errors="surrogateescape")):
                add("native_demo", name)
    add("native_template", "template_project_fmi2/demo_FmuComponentChrono.cpp")
    for row in json.loads((root / INPUT_MANIFESTS[1]).read_text())["programs"]:
        add("cuda_demo", row["source"].removeprefix(pins["fea"]["path"] + "/"), "fea", row)
    python = json.loads((root / INPUT_MANIFESTS[2]).read_text())
    admission = {r["source"]: r for r in json.loads((root / INPUT_MANIFESTS[3]).read_text())["programs"]}
    for row in python["programs"]:
        add("python_demo", row["source"], declaration=admission[row["source"]])
    for row in json.loads((root / INPUT_MANIFESTS[4]).read_text())["programs"]:
        add("csharp_demo", row["source"].removeprefix(pins["chrono_capabilities"]["path"] + "/"), declaration=row)
    counts = dict(collections.Counter(row["family"] for row in result))
    if counts != EXPECTED or len({r["source"] for r in result}) != sum(EXPECTED.values()):
        raise ValueError("Source denominator changed; review scope explicitly: " + str(counts))
    # This real original CLI is retained without silently enlarging the agreed
    # demo/template denominator. Additional applications belong in this section.
    add("application", "src/chrono_precice/yaml_app/run_chrono_adapter.cpp")
    return {"source_pins": pins, "counts": counts, "programs": result,
            "input_sha256": {name: digest((root / name).read_bytes()) for name in INPUT_MANIFESTS}}
