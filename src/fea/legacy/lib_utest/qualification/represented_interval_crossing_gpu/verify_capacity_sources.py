#!/usr/bin/env python3
"""Freeze the numerical kernel and launch while widening only retained capacity."""
from pathlib import Path
import hashlib
import json

here = Path(__file__).resolve().parent
root = here.parents[2]
collision = root / "lib_src/collision"
directory = collision / "represented_interval_crossing/native_device"
public = (collision / "RepresentedIntervalCrossingGpu.h").read_text()
types = (directory / "KernelTypes.h").read_text()
kernel = (directory / "Kernels.cu").read_text()
assert "device_workers = 128" in public
assert "MaximumDeviceWorkers = 4096" in types
assert "DefaultDeviceWorkers = RepresentedIntervalGpuLimits{}.device_workers" in types
assert "ThreadsPerBlock = 32" in types
# Exact source bytes at the qualified compound1157 predecessor. This includes
# the complete device kernel and Launch adapter; resource introspection follows.
unchanged = kernel[kernel.index("__global__ __launch_bounds__"):
                   kernel.index("cudaError_t QueryKernelResources")]
assert hashlib.sha256(unchanged.encode()).hexdigest() == "1aea26b18ddbb0ee6f154099aeb0a63d9a593b7a0cb0973d069e1b115d57b156"
assert "cudaDeviceSetLimit" not in kernel
assert "next.default_workers = DefaultDeviceWorkers" in kernel
assert "next.worker_limit = MaximumDeviceWorkers" in kernel
layout = (directory / "Layout.cpp").read_text()
assert "limits.device_workers > MaximumDeviceWorkers" in layout
assert "device.Append<ExactScratch>(limits.device_workers" in layout
assert "limits.device_workers * next.dfs_capacity" in layout
for name in ("CapacityHostTest.cpp", "CapacityCudaTest.cpp", "GpuFixture.h"):
    assert name in (here / "CMakeLists.txt").read_text()
    assert name in (here / "BUILD.bazel").read_text()
print(json.dumps({"status": "passed", "kernel_and_launch_bytes_unchanged_from": "1157dd8c",
                  "default_workers": 128, "maximum_workers": 4096,
                  "cuda_execution": False}))
