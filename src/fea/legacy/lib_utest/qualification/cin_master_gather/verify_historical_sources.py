#!/usr/bin/env python3
"""Authenticate the gather slice and every affected prior source identity."""
from pathlib import Path
import hashlib
import json
import runpy


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
MANIFEST_SHA256 = "0d3d5dfaab1ee369cb6fcb7fc7fc24a514a5e324a5073115afa338a19ab4cf43"
raw = (HERE / "source-manifest.json").read_bytes()
assert hashlib.sha256(raw).hexdigest() == MANIFEST_SHA256
manifest = json.loads(raw)
for row in manifest["files"]:
    path = Path(row["path"])
    assert not path.is_absolute() and ".." not in path.parts
    data = (ROOT / path).read_bytes()
    assert len(data) == row["bytes"], path
    assert hashlib.sha256(data).hexdigest() == row["sha256"], path

proof = runpy.run_path(str(HERE / "gather_proof.py"))
proof["prove"]()
prepare = runpy.run_path(str(HERE / "prepare_reference.py"))
reference = (HERE / "reference/ExplicitNodalCinStep.cu").read_text()
generated = prepare["render"](reference)
assert generated.count("cudaError_t LaunchFrozen(") == 1
assert "FENodalState::Impl::LaunchCinAdvance" not in generated
assert generated.endswith("} // namespace tl::fea::cin_gather_test\n")

package = (HERE / "BUILD.bazel").read_text()
for token in [
    'name = "frozen_caller"',
    'name = "fixture"',
    'name = "host"',
    'name = "cuda"',
    '"//lib_utest/qualification/cin_force_transfers:owner_test_source"',
    '"//lib_utest/qualification/cin_parallel_ordinary:device_packet_sources"',
]:
    assert package.count(token) == 1, token
assert package.index('name = "frozen_caller"') < package.index('name = "cuda"')
cmake = (HERE / "CMakeLists.txt").read_text()
assert cmake.count("cin_master_gather_sources") == 2
assert cmake.count("cin_master_gather_abi") == 2
assert cmake.count("CIN_MASTER_GATHER_CUDA") == 2
production_cmake = (ROOT / "lib_src/solvers/CMakeLists.txt").read_text()
production_bazel = (ROOT / "lib_src/solvers/BUILD.bazel").read_text()
assert production_cmake.count("cin_advance/ForceGather.cu") == 1
assert production_bazel.count('"cin_advance/ForceGather.cu"') == 1
visibility = '"//lib_utest/qualification/cin_master_gather:__pkg__"'
force_package = (HERE.parent / "cin_force_transfers/BUILD.bazel").read_text()
input_package = (HERE.parent / "cin_force_inputs/BUILD.bazel").read_text()
ordinary_package = (HERE.parent / "cin_parallel_ordinary/BUILD.bazel").read_text()
mapped_package = (ROOT / "lib_src/elements/mapped_shell/BUILD.bazel").read_text()
assert force_package.count(visibility) == 2
assert 'name = "owner_test_source"' in force_package
assert input_package.count(visibility) == 1
assert ordinary_package.count(visibility) == 2
assert mapped_package.count(visibility) == 1

prior_entrypoints = [
    "qeph_mapped_gather/verify_sources.py",
    "t3_mapped_gather/verify_sources.py",
    "qbat_mapped_gather/verify_sources.py",
    "cin_parallel_ordinary/verify_sources.py",
    "cin_force_inputs/verify_sources.py",
    "cin_parallel_screen/verify_sources.py",
    "cin_parallel_capture/verify_sources.py",
    "cin_parallel_groups/verify_sources.py",
    "cin_force_transfers/verify_sources.py",
    "cin_limiter/verify_sources.py",
    "cin_parallel_recovery/verify_sources.py",
    "cin_parallel_drift/verify_sources.py",
    "nodal_seal_rows/verify_sources.py",
    "nodal_reset_rows/verify_sources.py",
    "cin_cooperative_group_screen/verify_sources.py",
]
for relative in prior_entrypoints:
    runpy.run_path(str(HERE.parent / relative))

print(json.dumps({
    "status": "passed",
    "records": len(manifest["files"]),
    "baseline": manifest["baseline_commit"],
    "prior_entrypoints": len(prior_entrypoints),
    "numerical_execution": False,
    "cuda_compilation": False,
    "cuda_execution": False,
    "v5_device_bytes": 2679632,
    "v5_retained_host_bytes": 535924,
    "v5_temporary_host_bytes": 1507724,
    "per_step_allocations": 0,
}))
