"""Launch the real ROS node with only its declared middleware/runtime prefix."""

import os
from pathlib import Path
import sys

from python.runfiles import runfiles
from src.integrations.ros import NodePaths
from tools.ros.environment import ros_environment


def main():
    resolver = runfiles.Create()
    if resolver is None:
        raise RuntimeError("ROS node launch requires declared backend and SDK runfiles")
    backend = resolver.Rlocation(NodePaths.BACKEND)
    receipt = resolver.Rlocation(NodePaths.SDK_RECEIPT)
    if not backend or not receipt or not Path(backend).is_file() or not Path(receipt).is_file():
        raise RuntimeError("Declared ROS node or SDK is missing")
    root = Path(receipt).absolute().parent
    message_sdks = []
    for logical in NodePaths.MESSAGE_SDK_RECEIPTS:
        value = resolver.Rlocation(logical)
        if not value or not Path(value).is_file():
            raise RuntimeError("Declared custom ROS message SDK is missing")
        message_sdks.append(Path(value).absolute())
    environment = ros_environment(os.environ, root, message_sdks)
    # ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY intentionally remain caller-controlled
    # so tests can isolate their middleware domain without changing application
    # communication semantics. No global environment or ROS installation changes.
    os.execve(backend, [backend] + sys.argv[1:], environment)


if __name__ == "__main__":
    main()
