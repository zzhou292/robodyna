"""Run the unchanged ROS bridge round-trip suite with a real declared node."""

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile

from python.runfiles import runfiles
from tools.ros.environment import ros_environment


def main():
    if len(sys.argv) not in (4, 5):
        raise RuntimeError("Expected test executable, node, SDK and optional declared custom-message receipt")
    resolver = runfiles.Create()
    values = [resolver.Rlocation(name) for name in sys.argv[1:]]
    if any(not value or not Path(value).is_file() for value in values):
        raise RuntimeError("Declared ROS integration inputs are missing")
    test, node, sdk = [Path(value).absolute() for value in values[:3]]
    messages = [Path(value).absolute() for value in values[3:]]
    environment = ros_environment(os.environ, sdk.parent, messages)
    for name in ("ROS_DISCOVERY_SERVER", "FASTDDS_DEFAULT_PROFILES_FILE", "FASTRTPS_DEFAULT_PROFILES_FILE",
                 "RMW_FASTRTPS_USE_QOS_FROM_XML", "CYCLONEDDS_URI"):
        environment.pop(name, None)
    environment["ROS_LOCALHOST_ONLY"] = "1"
    environment["ROS_DOMAIN_ID"] = str(10 + int.from_bytes(os.urandom(2), "little") % 90)
    parent = os.environ.get("TEST_UNDECLARED_OUTPUTS_DIR") or os.environ.get("TEST_TMPDIR")
    root = Path(tempfile.mkdtemp(dir=parent, prefix="robodyna-ros-"))
    # Keep request, log and result on failure in Bazel's admitted output area.
    # FindExecutable's retained execv contract accepts this explicit cwd binding;
    # no old installation or ambient PATH is selected.
    (root / "chrono_ros_node").symlink_to(node)
    (root / "request.json").write_text(json.dumps({
        "schema": "robodyna.ros_bridge_test.v1", "test": str(test), "node": str(node),
        "sdk": str(sdk), "message_sdks": [str(value) for value in messages],
        "domain": environment["ROS_DOMAIN_ID"], "localhost_only": True,
    }, indent=2) + "\n")
    process = None
    result = {"status": "failed", "exit_code": None, "node_process_group_killed": False}
    try:
        with (root / "test.log").open("x") as log:
            process = subprocess.Popen([str(test)], cwd=root, env=environment, stdout=log,
                                       stderr=subprocess.STDOUT, start_new_session=True)
            result["exit_code"] = process.wait(timeout=240)
        if result["exit_code"]:
            raise RuntimeError("Real ROS bridge suite failed: " + str(result["exit_code"]))
        result["status"] = "passed"
    except BaseException as error:
        # On timeout the unreaped leader still reserves this owned process-group
        # identity. The retained node uses fork/exec and stays in that group.
        if process is not None and process.returncode is None:
            os.killpg(process.pid, signal.SIGKILL)
            result["node_process_group_killed"] = True
            result["exit_code"] = process.wait()
        result["error"] = str(error)
        raise
    finally:
        (root / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        log_path = root / "test.log"
        if log_path.exists():
            print(log_path.read_text())
        print("ROS integration evidence:", root)


if __name__ == "__main__":
    main()
