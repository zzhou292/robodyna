#!/usr/bin/env python3
"""Authenticate the frozen caller and exact unchanged stage arithmetic/order."""
from pathlib import Path
import hashlib
import json
import re

here = Path(__file__).resolve().parent
root = here.parents[2]
raw = (here / "source-manifest.json").read_bytes()
assert hashlib.sha256(raw).hexdigest() == "ebd82719851c8ec51ca5b73bd7d068fa663b0aee0cee18d54f98eddb99761405"
manifest = json.loads(raw)
for row in manifest["files"]:
    path = Path(row["path"])
    assert not path.is_absolute() and ".." not in path.parts
    value = (root / path).read_bytes()
    assert len(value) == row["bytes"], path
    assert hashlib.sha256(value).hexdigest() == row["sha256"], path

old = (here / "serial/ExplicitNodalCinStep.cu.txt").read_text()
current = (root / "lib_src/solvers/ExplicitNodalCinStep.cu").read_text()
extract = old[:old.index("cudaError_t FENodalState::Impl::LaunchCinAdvance")]
extract = extract.replace("__device__ void Fail", "TL_CIN_SERIAL_DEVICE void Fail")
extract = extract.replace("__global__ void AdvanceCin", "TL_CIN_SERIAL_KERNEL void AdvanceCin")
assert extract == (here / "serial/Kernel.inc").read_text()

start = "  if (control->status != NodalStatus::Ok) return;"
old_prefix = old[old.index(start):old.index("  auto* current_inverse")]
# The force-only input extraction now has its own exact-body proof. Keep the
# intermediate frozen owner prefix as the original ordinary-stage baseline.
import runpy
runpy.run_path(str(here.parent / "cin_force_inputs/verify_sources.py"))
intermediate = (here.parent / "cin_force_inputs/serial/ExplicitNodalCinStep.cu.txt").read_text()
prefix = intermediate[intermediate.index(start):intermediate.index("  *input.failure")]
assert old_prefix == prefix, "prefix arithmetic or error order changed"
suffix = "  // Source/owner admission proves CIN has no rigid member intersection."
without_capture = runpy.run_path(str(here.parent/"cin_parallel_capture/capture_proof.py"))["without_capture"]
assert without_capture(old[old.index(suffix):old.index("} // namespace")]) == current[
    current.index(suffix):current.index("} // namespace")], "suffix arithmetic or error order changed"

# Compare the extracted local node operation after only the explicit view-name
# substitutions and conversion from serial Fail/return to a returned status.
def compact(value):
    return re.sub(r"\s+", "", value)

old_node = old[old.index("    // There is no conventional inverse"):old.index(suffix)]
old_node = old_node[:old_node.rfind("  }")]
old_node = old_node.replace("continue;", "return NodalStatus::Ok;")
old_node = old_node.replace("Fail(control, NodalStatus::InvalidOutput, i);\n      return;",
                            "return NodalStatus::InvalidOutput;")
old_node = old_node.replace("const auto status = nodal_detail::", "return nodal_detail::")
old_node = old_node[:old_node.index("    if (status != NodalStatus::Ok)")]
node = (root / "lib_src/solvers/cin_advance/Node.h").read_text()
node = node[node.index("  // There is no conventional inverse"):node.index("\n}\n")]
for line in ["  const auto& groups = input.groups;\n", "  const auto* tail = input.tail;\n",
             "  const auto* fixed = input.fixed;\n", "  const auto* rotation_present = input.rotation_present;\n"]:
    node = node.replace(line, "")
for name in ("model", "accepted", "trial", "loads", "durations", "maximum_angle"):
    node = node.replace("input." + name, name)
node = node.replace("input.work+3*n", "acceleration")
node = node.replace("input.work+6*n", "angular_acceleration")
assert compact(old_node) == compact(node), "node operation changed beyond local status return"

# New scheduling controls: prefix success seeds the key, each worker owns a
# unique node, and only completion writes the shared status before the suffix.
ordinary = current[current.index("__global__ void AdvanceOrdinaryCin"):current.index("__global__ void CompleteCin")]
assert "if (input.control->status != NodalStatus::Ok) return;" in ordinary
assert "node += blockDim.x*gridDim.x" in ordinary
assert "atomicMin(input.failure, cin_advance::EncodeFailure(node, status));" in ordinary
completion = current[current.index("__global__ void CompleteCin"):current.index(suffix)]
assert completion.index(start) < completion.index("const auto failure = *input.failure;")
assert "Fail(control, cin_advance::FailureStatus(failure), cin_advance::FailureNode(failure));" in completion
print(json.dumps({"status": "passed", "baseline": manifest["baseline_commit"],
                  "records": len(manifest["files"]),
                  "unchanged": ["prefix", "ordinary arithmetic", "rigid/recovery/drift/capture suffix"],
                  "numerical_execution": False}))
