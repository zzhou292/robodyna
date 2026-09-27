#!/usr/bin/env python3
"""Literal old/current GPU kernel bodies; no arithmetic or status adaptation."""
from pathlib import Path
import argparse
import hashlib
import json
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
MANIFEST = "60dd0ca7d152a952044bc773d10cde23dd398892dd8b466bd572e944fa89ec74"
SOURCE = "lib_src/collision/radioss_type25/runtime/Kernels.cu"

def once(text, old, new):
    assert text.count(old) == 1, (old, text.count(old))
    return text.replace(old, new, 1)

def function(text, begin, end):
    assert text.count(begin) == 1 and text.count(end) == 1
    return text[text.index(begin):text.index(end)]

def transformed(text):
    marker = "  // Diagnostic only; no atomics/reassociated native work sums or host packets.\n"
    old = function(text, "__global__ void Diagnostics(", "__global__ void GatherNodes(")
    body = once(old, marker, marker + "  auto active=d.control->active;\n"
        "  double elastic_energy=d.control->elastic_energy,damping_work=d.control->damping_work;\n"
        "  double friction_work=d.control->friction_work;\n")
    begin = body.index("  for(std::size_t i=0;")
    end = body.index("  if(!tl::math::Finite(")
    fold = body[begin:end]
    for field in ("active", "elastic_energy", "damping_work", "friction_work"):
        fold = fold.replace("d.control->" + field, field)
    body = body[:begin] + fold + "  d.control->active=active;d.control->elastic_energy=elastic_energy;\n" \
        "  d.control->damping_work=damping_work;d.control->friction_work=friction_work;\n" + body[end:]
    return once(text, old, body)

def checked():
    raw = (HERE / "baseline.json").read_bytes()
    assert hashlib.sha256(raw).hexdigest() == MANIFEST
    data = json.loads(raw)
    assert data["commit"] == "2733b4ccb916db96fc4d7d51cdfa21566d6e227d"
    for base, entries in ((HERE, [data["frozen"]]), (ROOT, data["unchanged"])):
        for row in entries:
            raw = (base / row["path"]).read_bytes()
            assert len(raw) == row["bytes"] and hashlib.sha256(raw).hexdigest() == row["sha256"], row["path"]
    old = (HERE / data["frozen"]["path"]).read_text()
    current = (ROOT / SOURCE).read_text()
    assert current == transformed(old), "Complete production translation-unit reversal"
    return old, current

def generate(output):
    old, current = checked()
    output.mkdir(parents=True, exist_ok=True)
    sections = ["#pragma once", '#include "lib_src/collision/radioss_type25/runtime/Layout.h"']
    for name, source in (("reference", old), ("current", current)):
        sections.append("namespace tlfea::contact::radioss_type25::runtime_detail::diagnostic_" + name + " {")
        sections.append(function(source, "__device__ void Fail(", "__global__ void Reset("))
        sections.append(function(source, "__global__ void Diagnostics(", "__global__ void GatherNodes("))
        sections.append("}")
    (output / "DiagnosticBodies.h").write_text("\n".join(sections) + "\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    checked()
    if args.output:
        generate(args.output)
    print(json.dumps({"status": "passed", "exact_terminal_aggregate_transformation": True,
        "other_kernels_launch_order_layout_and_owner_unchanged": True}))
