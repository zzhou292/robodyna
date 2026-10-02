# ROS integration ownership

The retained implementation separates simulation from middleware. The simulation
library owns the existing manager, handlers and shared-memory bridge; it links
the current mechanics backend and contains no ROS2 middleware library. A separate
real C++ node owns rclcpp, type support and DDS. The protocol layer is shared
between them and has neither a mechanics nor ROS2 dependency.

`sources.bzl` and `SourceBaseline.json` inventory all 27 original implementation
files in eight source families. Current native owners expose the seven protocol
units, five simulation API units, four basic handlers, and the separate vehicle
and robot handlers. The node has exactly its original two implementation units.
Sensor/rendered-sensor and URDF admission remain dependent on their actual module
profiles; metadata does not stand in for those implementations.

The four original protocol tests passed under the workspace guard in
`crash-work/reports/robodyna-ros-protocol-build-1.json`. They exercise CDR bytes,
message/schema behavior, transport wraparound and real shared-memory/threaded
exchange. The old development script's fake-header syntax stage is not used.

```text
//src/integrations/ros:protocol_tests
//src/integrations/ros:simulation
//src/integrations/ros:node_backend
//src/integrations/ros:node
//src/integrations/ros:bridge_test
```

The `node` alias runs the real node through a declared SDK environment. It keeps
the inherited `chrono_ros_node` executable basename for process compatibility.
The retained process launcher uses `execv`, so its apparent PATH fallback does
not search PATH. Admitted launchers bind the selected executable explicitly in
their create-only working directory; they do not select an old installation.
Generated runfile identities are embedded in the launcher configuration because
`py_binary(args=...)` applies to `bazel run`, not arbitrary native `execv` callers.

The bridge integration target reuses the original C++ round-trip tests with real
rclcpp peers and the actual node. It restricts discovery to localhost and chooses
a separate valid ROS domain for each test process, removing inherited discovery
server/profile overrides. It preserves logs and kills its owned test/node process
group on timeout. The outer workstation guard remains required. These test-only
settings do not alter interactive demo communication semantics.

Node compilation and the bridge integration target are pending their next closed
receipts. A successful protocol test does not qualify DDS discovery, ROS message
support, every native/Python/C# example or GPU sensor rendering.

The official `chrono_ros_interfaces` package supplies the custom vehicle/Viper
message types. It is a separate required generated-message dependency; those
demos must not be called runtime-ready until its real type-support libraries are
available. ROS Humble's DDS libraries also remain separate from SynChrono's
distinct, explicitly pinned standalone DDS profile.
