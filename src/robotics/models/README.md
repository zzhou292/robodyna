# Retained robot model library

`//src/robotics/models:models` compiles the original 16 C++ translation units in
seven CMake source groups: actuation, RoboSimian, Curiosity, Viper, Turtlebot,
Little Hexy, and industrial robots/kinematics/trajectory interpolation. Source
locations and APIs are retained. The library uses the existing mechanics owner;
it does not compile Vehicle, Vehicle models, FSI, or a second System implementation.

`CH_API_COMPILE_MODELS` stays private. The initial library is CPU mechanics with
the qualified core configuration. The industrial classes named `CAD` operate on
already imported bodies and markers and do not themselves require OpenCascade.
`RoboSimianURDF.cpp` and its header remain explicitly pending the native Parsers
URDF closure; they are not compiled by an indiscriminate source glob.

The source test checks the retained CMake groups, optional URDF exclusion, all 17
demo declarations, private include closure, and the pinned input bytes. The Bazel
analysis test checks actual model and Lander helper compile owners. The host test
exercises SCARA forward/inverse kinematics, its existing System ownership, and
joint trajectory interpolation without graphics, GPU allocation, or stepping.
Passing these gates does not qualify every robot model's dynamics or assets.

See [the demo catalog](../../../examples/robotics/DemoCatalog.json) for the complete
admission boundary and [runtime prerequisites](../../../examples/robotics/RUNTIME_ASSETS.md)
before launching a demo.
