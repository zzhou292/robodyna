#!/usr/bin/env python3
"""Owning source-shape proof for the optional native GPU facade; no numerical execution."""
from pathlib import Path
import re
root = Path(__file__).resolve().parents[3]
collision = root / "lib_src/collision"
public = (collision / "RepresentedIntervalCrossingGpu.h").read_text()
facade = (collision / "RepresentedIntervalCrossingGpu.cpp").read_text()
seam = (collision / "represented_interval_crossing/DeviceExecution.h").read_text()
kernel = (collision / "represented_interval_crossing/native_device/Kernels.cu").read_text()
workspace = (collision / "represented_interval_crossing/native_device/Workspace.cpp").read_text()
for token in ("RepresentedIntervalCrossingGpu", "device_workers = 128", "host_pairs", "device_pairs"):
    assert token in public, token
assert "class AuthenticatedWork" in seam
assert "AuthenticatedWork(const AuthenticatedWork&) = delete" in seam
assert "friend class ::tlfea::contact::RepresentedIntervalCrossing" in seam
assert "DeviceAccess::Certify" in facade
assert "NativeStorageDomain::FromPaths" in kernel
assert "cudaDeviceSetLimit" not in kernel + workspace + facade
assert "atomicAdd" not in kernel
assert "__global__" in kernel
here = Path(__file__).resolve().parent
for name in ("HostTest.cpp", "GpuTest.cpp", "Cases.h"):
    assert name in (here / "CMakeLists.txt").read_text()
assert len(re.findall(r"TEST\(NativeGpuForecast,", (here / "HostTest.cpp").read_text())) == 3
assert len(re.findall(r"TEST\(NativeGpuCuda,", (here / "GpuTest.cpp").read_text())) == 9
print("PASS standalone native GPU ownership/source shape; no numerical execution")
