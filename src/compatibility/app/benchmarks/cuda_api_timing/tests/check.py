"""CPU-only forwarding/interposition tests; never loads the real CUDA runtime."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

library, probe = map(lambda p: str(Path(p).resolve()), sys.argv[1:])
with tempfile.TemporaryDirectory(prefix="robo-dyna-cuda-api-timing-") as temporary:
    root = Path(temporary)
    def run(path=None, preload=True, early=False):
        env = os.environ.copy()
        env.pop("ROBO_DYNA_CUDA_TIMING_OUTPUT", None)
        env.pop("LD_PRELOAD", None)
        env.pop("ROBO_DYNA_FAKE_CUDA_EARLY_CALL", None)
        env["LD_LIBRARY_PATH"] = str(Path(probe).parent)
        if preload:
            env["LD_PRELOAD"] = library
        if path is not None:
            env["ROBO_DYNA_CUDA_TIMING_OUTPUT"] = str(path)
        if early:
            env["ROBO_DYNA_FAKE_CUDA_EARLY_CALL"] = "1"
        result = subprocess.run([probe], env=env, capture_output=True, text=True, timeout=10)
        assert result.returncode == 0, (result.returncode, result.stdout, result.stderr)
        assert result.stdout == "fake CUDA forwarding passed\n"
        return result

    run(preload=False)
    assert not run().stderr  # Disabled preload forwards without a report.
    assert not run("").stderr
    assert list(root.iterdir()) == []
    path = root / "timing.json"
    assert not run(path).stderr
    assert path.stat().st_size <= 65536
    report = json.loads(path.read_text())
    assert report["schema"] == "robo_dyna.cuda_api_timing.v1"
    assert report["process_interval_ns"] > 0
    assert report["active_calls_at_shutdown"] == 0
    assert report["nested_calls_skipped"] == 1
    assert report["missing_symbols"] == report["clock_failures"] == 0
    assert not report["counter_saturated"]
    functions = {item["name"]: item for item in report["functions"]}
    for name, calls, failures in [("cudaMemcpyAsync", 403, 1), ("cudaStreamSynchronize", 2, 1),
                                  ("cudaMemcpy", 2, 1), ("cudaDeviceSynchronize", 1, 0)]:
        value = functions[name]
        assert (value["calls"], value["failures"]) == (calls, failures)
        assert 0 < value["maximum_ns"] <= value["wall_ns"]
    value = functions["cudaMemcpyAsync"]
    assert value["requested_bytes"] == dict(host_to_host=4400, host_to_device=7, device_to_host=13,
                                            device_to_device=0, runtime_default=4, unknown=0)
    assert value["successful_bytes"] == dict(value["requested_bytes"], device_to_host=0)
    assert functions["cudaMemcpy"]["requested_bytes"]["device_to_device"] == 9
    assert functions["cudaMemcpy"]["requested_bytes"]["unknown"] == 3
    assert functions["cudaMemcpy"]["successful_bytes"]["unknown"] == 0
    original = path.read_bytes()
    assert "create-only report" in run(path).stderr
    assert path.read_bytes() == original
    link = root / "symlink.json"
    link.symlink_to(path)
    assert "create-only report" in run(link).stderr
    assert path.read_bytes() == original and link.is_symlink()
    assert "path exceeds bound" in run("x" * 4096).stderr
    assert "create-only report" in run(root / "absent" / "timing.json").stderr
    assert not run(early=True).stderr
    early_path = root / "early.json"
    assert not run(early_path, early=True).stderr
    early_report = json.loads(early_path.read_text())
    for early_value, value in zip(early_report["functions"], report["functions"]):
        for key in ("name", "calls", "failures", "requested_bytes", "successful_bytes"):
            assert early_value[key] == value[key], (key, early_value, value)
    assert early_report["missing_symbols"] == early_report["active_calls_at_shutdown"] == 0
print("PASS: baseline, disabled, forwarding, failures/errno, recursion, concurrency, bytes, create-only, symlink/path bounds and early constructor forwarding")
