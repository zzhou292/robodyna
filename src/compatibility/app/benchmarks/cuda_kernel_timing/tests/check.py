"""CPU fake-runtime checks. The real CUDA library is never loaded."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

library, probe = [str(Path(path).resolve()) for path in sys.argv[1:]]
with tempfile.TemporaryDirectory(prefix="robo-dyna-kernel-timing-") as temporary:
    root = Path(temporary)

    def run(path=None, mode=None, preload=True, early=False, overflow=False):
        env = os.environ.copy()
        for name in ("LD_PRELOAD", "ROBO_DYNA_CUDA_KERNEL_TIMING_OUTPUT", "FAKE_CUDA_MODE", "FAKE_CUDA_EARLY"):
            env.pop(name, None)
        env["LD_LIBRARY_PATH"] = str(Path(probe).parent)
        if preload:
            env["LD_PRELOAD"] = library
        if path is not None:
            env["ROBO_DYNA_CUDA_KERNEL_TIMING_OUTPUT"] = str(path)
        if mode:
            env["FAKE_CUDA_MODE"] = mode
        if early:
            env["FAKE_CUDA_EARLY"] = "1"
        command = [probe] + (["overflow"] if overflow else [])
        result = subprocess.run(command, env=env, text=True, capture_output=True, timeout=15)
        assert result.returncode == 0, (result.returncode, result.stdout, result.stderr, mode)
        assert result.stdout == "fake kernel forwarding passed\n"
        return result

    run(preload=False)
    assert not run().stderr
    assert not run("").stderr
    assert not list(root.iterdir())
    path = root / "profile.json"
    assert not run(path).stderr
    report = json.loads(path.read_text())
    assert report["schema"] == "robo_dyna.cuda_kernel_timing.v1"
    assert report["diagnostic_serialization"]
    assert report["active_at_shutdown"] == report["instrumentation_failures"] == 0
    assert report["invalid_event_times"] == report["registrations_dropped"] == report["handles_dropped"] == 0
    assert not report["counter_saturated"]
    assert report["public_launch_calls"] == 1 and report["cuda13_handle_launch_calls"] == 45
    entries = {entry["name"]: entry for entry in report["kernels"]}
    original = entries['kernel_"line\\\n']
    assert original["calls"] == 44 and original["launch_failures"] == 1
    assert original["timed_calls"] == 43 and original["unmeasured_calls"] == 1
    assert original["total_device_ns"] == 43 * 1250000
    assert original["maximum_device_ns"] == original["first_timed_device_ns"] == 1250000
    assert original["first_grid"] == [1, 2, 3] and original["first_block"] == [4, 5, 6]
    assert original["first_shared_bytes"] == 19 and original["dimensions_changed_calls"] == 1
    assert not original["active_registration"]
    assert report["unknown"]["calls"] == report["unknown"]["timed_calls"] == 1
    assert entries["replacement_kernel"]["calls"] == 1
    assert report["fixed_state_bytes"] < 4 * 1024 * 1024
    assert path.stat().st_size <= report["report_byte_cap"]
    prior = path.read_bytes()
    assert "create-only report" in run(path).stderr
    assert path.read_bytes() == prior
    symlink = root / "symlink.json"
    symlink.symlink_to(path)
    assert "create-only report" in run(symlink).stderr
    assert path.read_bytes() == prior and symlink.is_symlink()
    assert "path exceeds bound" in run("x" * 4096).stderr
    assert "create-only report" in run(root / "absent" / "profile.json").stderr

    for mode in ("capture", "capture_error", "create_error", "start_error", "stop_error",
                 "sync_error", "elapsed_error", "destroy_error", "invalid_time", "nested"):
        destination = root / (mode + ".json")
        assert not run(destination, mode).stderr
        value = json.loads(destination.read_text())
        assert value["public_launch_calls"] + value["cuda13_handle_launch_calls"] == 46
        assert value["active_at_shutdown"] == 0
        if mode == "capture":
            assert value["capture_calls_skipped"] == 46
            assert value["instrumentation_failures"] == 0
            assert all(entry["timed_calls"] == 0 for entry in value["kernels"])
        elif mode == "invalid_time":
            assert value["invalid_event_times"] == 45
        elif mode == "nested":
            assert value["nested_calls_skipped"] == 45
            assert value["instrumentation_failures"] == 0
        else:
            assert value["instrumentation_failures"] > 0

    early_path = root / "early.json"
    assert not run(early_path, early=True).stderr
    early = json.loads(early_path.read_text())
    assert any(entry["name"] == "early_kernel" for entry in early["kernels"])
    unnamed_path = root / "unnamed.json"
    assert not run(unnamed_path, "unnamed").stderr
    unnamed = json.loads(unnamed_path.read_text())
    assert unnamed["unknown"]["calls"] == 45 and unnamed["registrations_dropped"] == 1
    assert all(entry["name"] for entry in unnamed["kernels"])
    cap_path = root / "cap.json"
    assert not run(cap_path, overflow=True).stderr
    cap = json.loads(cap_path.read_text())
    assert len(cap["kernels"]) == cap["registry_capacity"] == 2048
    assert cap["registrations_dropped"] > 0 and cap["handles_dropped"] > 0
    assert any(entry["name_truncated"] for entry in cap["kernels"])
    assert cap_path.stat().st_size <= cap["report_byte_cap"]
print("PASS: disabled/baseline, CUDA13/public forwarding, exact results/errno, names/handles, "
      "unregistration/rebinding, bounded counters, concurrency, nested calls, early registration, "
      "capture, every event failure, invalid time, device lifetime and create-only output")
