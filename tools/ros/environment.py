"""Controlled middleware lookup for the declared ROS subprocess SDK."""

import os
import json
from pathlib import Path

from tools.bindings.run_swig import tool_environment


def _message_prefixes(receipts):
    prefixes = []
    packages = set()
    for receipt in receipts:
        receipt = Path(receipt).absolute()
        record = json.loads(receipt.read_text())
        if record.get("schema") != "robodyna.ros_interfaces_sdk.v1" or record.get("prefix") != ".":
            raise ValueError("Unsupported declared ROS message SDK")
        package = record.get("package")
        if package != "chrono_ros_interfaces" or package in packages:
            raise ValueError("Unadmitted or duplicate ROS message package")
        packages.add(package)
        marker = receipt.parent / "share/ament_index/resource_index/packages" / package
        if not marker.is_file():
            raise ValueError("Declared message SDK lost its ament package marker")
        prefixes.append(receipt.parent)
    return prefixes


def ros_environment(environment, sdk_root, message_sdks=()):
    root = Path(sdk_root).absolute()
    prefix = root / "opt/ros/humble"
    messages = _message_prefixes(message_sdks)
    result = tool_environment(environment)
    for name in ("COLCON_PREFIX_PATH", "CMAKE_PREFIX_PATH", "LD_PRELOAD"):
        result.pop(name, None)
    result.update({
        "AMENT_PREFIX_PATH": os.pathsep.join(str(path) for path in messages + [prefix]),
        "LD_LIBRARY_PATH": os.pathsep.join(str(path) for path in
                                         [path / "lib" for path in messages] +
                                         [root / "lib", prefix / "lib", root / "usr/lib/x86_64-linux-gnu"]),
        "ROS_DISTRO": "humble",
        "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
    })
    return result
