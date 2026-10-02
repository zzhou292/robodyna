# Runtime packaging after compilation

These original interactive demos are build targets, not bounded launch recipes.
The next runtime gate must use a fresh output directory and the existing resource
guard, preserve each original timestep, and resolve its declared data tree before
constructing models or graphics. The core and Vehicle data paths are separate
stores; a core data path does not automatically configure Vehicle data.

| Demo family | Source-grounded asset roots and additional requirements |
| --- | --- |
| Curiosity | `data/robot/curiosity/obj`, `col`, and `rocks`; mesh basenames are constructed in `Curiosity.cpp`; rigid ground uses `data/textures/concrete.jpg` |
| Viper | `data/robot/viper/obj` and `col`; wheel variants select different basenames in `Viper.cpp`; CRM helpers additionally use `nasa_viper_wheel.obj` |
| Turtlebot | `data/robot/turtlebot`; `Turtlebot.cpp` constructs mesh names; ground uses the concrete texture |
| Little Hexy | `data/robot/copters/hexi_body.obj` and `prop.obj`, their material dependencies, and the ground texture |
| RoboSimian | `data/robot/robosimian` collision/visual meshes and the selected actuation data files; retain pose/start/cycle/stop conventions and the demo's output paths |
| Industrial | Preserve the selected model/setup in the original main. CAD model classes need the referenced bodies and markers already present in the System; class compilation alone does not import a CAD assembly |
| Lander | Original geometric helper builds its own bodies; graphics needs the standard VSG asset group. The CRM case additionally initializes the actual SPH solver and requires GPU runtime admission |
| Pending Sensor cases | Also require the chosen Sensor backend, compiled shader/runtime assets, and environment textures, including `sensor/textures/starmap_2020_4k.hdr` for Viper |

An eventual asset manifest must follow OBJ `mtllib` entries and MTL textures,
JSON-selected model/mesh references, and glTF external buffers/images if present.
It must preserve relative paths rather than copying only the filenames visible
in each `main`. Model constructors can load collision meshes before visualization
is initialized; a headless run may still need model assets. User-selected files
and model variants must be part of the launch manifest instead of silently
falling back to an ambient data directory.

Standard VSG/Irrlicht resources already come from the corresponding native feature
providers. No new robot asset-completeness or dynamics result is recorded by this
compile-only batch. A later guarded run must establish both before marking a
catalog entry runtime-qualified.
