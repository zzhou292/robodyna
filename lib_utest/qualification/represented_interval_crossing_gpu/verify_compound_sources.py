#!/usr/bin/env python3
"""Source-only ownership checks for the native CUDA compound scene lease."""
from pathlib import Path
import json

here = Path(__file__).resolve().parent
root = here.parents[2]
collision = root / "lib_src/collision"
native = (collision / "RepresentedIntervalCrossing.cpp").read_text()
lease = (collision / "represented_interval_crossing/DeviceExecution.h").read_text()
batch = (collision / "represented_interval_crossing/BatchExecution.h").read_text()
workspace = (collision / "represented_interval_crossing/native_device/Workspace.cpp").read_text()
storage = (collision / "represented_interval_crossing/native_device/Workspace.h").read_text()
facade = (collision / "RepresentedIntervalCrossingGpu.cpp").read_text()
tests = (here / "CompoundCudaTest.cpp").read_text()
assert "AuthenticatedScene(const AuthenticatedScene&) = delete" in lease
assert "AuthenticatedScene& operator=(const AuthenticatedScene&) = delete" in lease
assert "struct ConstructionKey" in lease and "ConstructionKey() = default" in lease
assert "DeviceExecution* const executor_" in lease
assert "std::optional<represented_interval_crossing::AuthenticatedScene> device_scene" in native
assert native.count("roster.device_scene.emplace(") == 1
slice_body = native[native.index("RepresentedIntervalReport RepresentedIntervalCrossing::Impl::CertifySlice("):
                    native.index("represented_interval_crossing::BatchAccess::Certify(")]
assert slice_body.index("roster.authenticated = true") < slice_body.index("roster.device_scene.emplace(")
assert slice_body.index("storage.staging.resize(storage.pairs.size())") < slice_body.index("roster.device_scene.emplace(")
assert "const represented_interval_crossing::AuthenticatedWork work(*roster.device_scene" in slice_body
assert "storage.CertifySlice(paths, path_count, slice, count, roster, device)" in native
assert "return CertifyUsing(crossing, paths, path_count, pairs, pair_count," in native
assert batch.index("!Bytes(paths, path_count") < batch.index("!extra_admission(") < batch.index("RepresentedIntervalPairKey preceding")
assert "device->Disjoint(scratch, scratch_bytes)" in native
assert "work.scene_.executor_ != this" in workspace
assert workspace.count("if (!work.scene_.uploaded_)") == 1
assert workspace.count("work.scene_.uploaded_ = true") == 1
assert workspace.index("cudaStreamSynchronize(stream_)") < workspace.index("work.scene_.uploaded_ = true")
assert workspace.index("PairExecution::Complete") < workspace.index("work.scene_.uploaded_ = true")
assert "report_.device_pairs += jobs" in workspace
for forbidden in ("AuthenticatedScene*", "AuthenticatedScene&", "RepresentedTrianglePath*", "scene_ready", "scene_generation"):
    assert forbidden not in storage, forbidden
assert facade.index("owner.busy.compare_exchange_strong") < facade.index("owner.workspace.BeginAttempt")
for name in (
    "OneAuthenticationAndUploadAcross256And257Boundary",
    "WideOnlyAndInitiallyWideSlicesDoNotUploadEarly",
    "EmptyCallsAndReusedInputAddressesCannotBorrowOldScene",
    "MalformedUnusedPathsRejectBeforeAnyDeviceWorkAndRetry",
    "LateWorkFailurePreservesLastSliceAndFreshRetryUpload",
    "OwnedScratchAndFacadeAliasesRejectBeforeBorrowedReads",
    "PairOrderAndCapacityErrorsKeepNativePriorityAndPublication",
):
    assert name in tests, name
print(json.dumps({"status": "passed", "cuda_execution": False,
                  "scope": "native lexical scene and shared compound publication ownership"}))
