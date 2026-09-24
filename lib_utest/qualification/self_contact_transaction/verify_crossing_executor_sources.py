#!/usr/bin/env python3
"""Source architecture gate for optional native CUDA routing; no execution."""
from pathlib import Path
root = Path(__file__).resolve().parents[3]
c = root / "lib_src/collision"
unit = c / "self_contact_transaction"
header = (unit / "CrossingExecutor.h").read_text()
owner = (unit / "CrossingExecutor.cpp").read_text()
values = (unit / "CrossingExecutorValues.cpp").read_text()
candidate = (unit / "Candidate.cpp").read_text()
config = (c / "SelfContactTransactionTypes.h").read_text()
assert "enable_cuda_native_crossing = false" in config
assert "native_crossing_device_workers = 128" in config
assert "native_crossing_numeric_cohort_pairs = 0" in config
assert owner.count("cpu_.Initialize(") == owner.count("gpu_.Initialize(") == 1
assert "else\n    result.native = cpu_.Initialize" in owner
assert "DeviceBatchAccess::Certify(gpu_" in owner
assert "CertifyCrossingBatches(cpu_" in owner
assert "if (use_gpu_)" in owner and "} else {" in owner
assert "gpu.native = limits.crossing;" in values
assert "if (!config.enable_cuda_native_crossing)" in values
assert "- sizeof(RepresentedIntervalCrossingGpu)" in values
assert "RepresentedIntervalCrossing cpu_;" in header
assert "RepresentedIntervalCrossingGpu gpu_;" in header
assert candidate.count("state.crossing.Certify(") == 2
assert "diagnostics.CrossingDevice(execution.device)" in candidate
assert "DescribeCrossingDeviceFailure" in candidate
forecast = (c / "RepresentedIntervalCrossingGpuValues.cmake").read_text()
assert "native_device/Forecast.cpp" in forecast
assert "native_device/Layout.cpp" in forecast
assert "CUDA::cudart" not in forecast and "enable_language" not in forecast
assert "native_device/OwnerStorage.h" in (c / "RepresentedIntervalCrossingGpu.cpp").read_text()
for name in ("CrossingExecutor.h", "CrossingExecutor.cpp", "CrossingExecutorValues.cpp"):
    for wiring in (c / "SelfContactTransaction.cmake", c / "BUILD.bazel"):
        assert name in wiring.read_text(), (name, wiring)
cmake = (Path(__file__).parent / "CMakeLists.txt").read_text()
for name in ("CrossingExecutorValueTest.cpp", "CrossingExecutorCudaCases.h",
             "NativeCrossingTransactionCudaCases.h", "CrossingForecastLink.cpp"):
    assert name in cmake, name
link = cmake[cmake.index("add_executable(self_contact_crossing_forecast_link"):
             cmake.index("add_executable(self_contact_transaction_medium_coupon")]
assert "-O0" in link and "CUDA::cudart" not in link
assert "tl_represented_interval_crossing_gpu_values" in link
print("PASS optional native execution owner, original CPU default, shared forecasts and bounded routing")
