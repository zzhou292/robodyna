# ROS2 node SDK

Set `ROBODYNA_ROS_ROOT` to the declared workspace extraction containing
`opt/ros/humble`, its Ubuntu support libraries and extraction receipt. This is
the Linux x86_64 Humble profile for Ubuntu22.04. There is no global ROS install,
shell startup change or package-maintainer script execution.

`ros_downloads.json` pins 78 ROS packages plus eight Ubuntu support packages:
8,420,058 compressed bytes. The extracted receipt records 6,821 files. SDK
admission authenticates selected headers, package resources, SONAMEs, exact
vendor libraries and their dependencies against `ros_pins.json`.

The source repository is the official ROS build-farm HTTPS mirror,
`https://repo.ros2.org/ubuntu/main/`. The normal packages.ros.org endpoint failed
certificate hostname validation during metadata lookup; no certificate check was
bypassed. ROS rclcpp is 16.0.21, FastDDS is 2.6.12 and FastCDR is 1.0.29. These
libraries are for the separate ROS node, not a replacement for SynChrono's older
standalone middleware profile.

`@ros_sdk//:node_sdk` follows the actual exported CMake header and link closure.
Generator expressions are resolved for the selected Linux/shared-library profile,
with original expressions and authenticated witness files recorded in the pins.
In particular, shared FastCDR requires `FASTCDR_DYN_LINK`; static YAML and false
empty-BOOL definitions remain disabled. Unresolved generator-expression strings
are rejected before reaching the C++ compiler.

`@ros_sdk//:runtime` includes ament resources and the real DDS/message/plugin
libraries. The node launcher selects that declared prefix and library search
directories; it does not source a global setup script. System glibc, libstdc++,
OpenSSL and the already available CPython3.10 development library remain explicit
platform prerequisites of this package profile. Portability to another platform
or ROS distribution requires separate SDK/runtime qualification.

The first node build exposed missing logical ownership of its private header and
unresolved CMake generator expressions. Both packaging corrections preserve the
original implementation sources. Their failed receipts remain diagnostic evidence;
only later closed gates can establish compilation and real bridge operation.

See [`src/integrations/ros/README.md`](../../src/integrations/ros/README.md) for
native ownership, process separation and the real integration-test contract.
