# Physical accepted replay

`Scene` consumes the immutable physical-run reader, keeps physical scale, and
publishes only exact archived samples. It reuses the source shell mapping,
0/1/3/4-point native plasticity applicability, stable original PID colors, and
actual activity. Its wall is the authenticated selected finite mesh. No physics
is advanced and no positions or plasticity are interpolated.

The application forecast includes the retained reader, presentation geometry and
sample workspace; VSG/Vulkan buffers and PNG encoder/decoder memory remain
separate measured render obligations. The scene default is one GiB, with a named
host cap up to two GiB. Failure to initialize or seek preserves prior publication.
A whole-archive scan fixes camera bounds and the plastic scale across samples.
An all-zero native plastic field uses a display maximum of one; it remains an
actual zero field. Missing applicability is never converted to a numeric zero.

Build the standalone CXX-only project here with the qualified TL helper root,
Chrono install and existing VSG packages. It does not enable CUDA or link the
vehicle dynamics factory. The owning targets are:

- `robo_dyna_physical_replay_scene`
- `robo_dyna_physical_scene_check` (`physical_scene_values`, optional `physical_scene_archive`)
- `robo_dyna_physical_scene_legacy_check` (`physical_scene_legacy`)
- `robo_dyna_physical_viewer_values_check` (`physical_viewer_values`)
- `robo_dyna_physical_replay` when `ROBO_DYNA_PHYSICAL_REPLAY_VIEWER=ON`

Use `Chrono_DIR=crash-work/install/chrono-vsg-r1/lib/cmake/Chrono` and
`CMAKE_PREFIX_PATH=crash-work/install/vsg-r0`, as absolute paths. The preserved
Vulkan/XCB development sysroot is under
`crash-work/dependencies/vsg-r0/ubuntu-dev/sysroot`; supply its Vulkan include /
library and pkg-config lib/share directories exactly as in the retained
`chrono-vsg-configure-1.json` root environment. Core-only checks do not need VSG.
Set `ROBO_DYNA_PHYSICAL_REPLAY_TESTS=OFF` for a viewer build without GTest.
The standalone root finds VSG dependencies before importing Chrono's VSG target
so the complete link interface is visible in the same CMake scope.

The CLI accepts one controller result directory or its `viewer-input.json`:

```
robo_dyna_physical_replay RUN_DIR
robo_dyna_physical_replay RUN_DIR --capture NEW_OUTPUT --color part-id
robo_dyna_physical_replay RUN_DIR --capture NEW_OUTPUT --color plastic-strain
```

Optional flags: `--view incident-side|wall-side`, `--fps 1..60`, `--wireframe`,
`--require-frames N`, `--receipt-sha256 SHA`, `--capture-cap-gib 2|6`.
To frame a detail, supply both `--camera-eye X,Y,Z` and `--camera-target X,Y,Z`
in the archive's world coordinates in metres. For example:

```
robo_dyna_physical_replay RUN_DIR --camera-eye -1.2,-1.8,1.0 --camera-target 0.2,0,0.5 --capture NEW_OUTPUT
```

The optional `--camera-up y|z` uses Chrono's existing axis-up camera; Z is the
default. It requires the complete eye/target pair. Explicit cameras cannot be
combined with `--view`. Nonfinite coordinates, coincident eye/target, an
up-parallel or unrepresentable camera basis are rejected. The fixed camera and
40-degree field of view change only presentation; archive geometry is unchanged.
The capture's existing position/target/FOV fields record the actual camera, with
`camera_view: explicit-fixed` and its actual Y/Z vertical. Default captures keep
their existing camera fields and values. No input receipt schema changes.

Playback fps selects how quickly stored samples are displayed; accepted times
remain their actual archive times. Initial-only and stopped accepted prefixes
are labeled as such. No deformation scaling or interpolation flag exists.

Capture uses the existing paired-render rule, one loading worker and a fixed
camera. All decoded PNGs, exact sample epochs/times, PID palette, selected wall
receipt, explicit channel availability, source receipt and completion/prefix
status are recorded in the capture manifest. A capture I/O failure leaves
incomplete evidence and no success manifest. The immutable binary input archive
is never modified. PNGs have a separate conservative 32-MiB/sample plus four-MiB
metadata forecast, default two GiB; six GiB is explicitly selected only if needed.

Root gates: set `ROBO_DYNA_PHYSICAL_REPLAY_INPUT` to a preserved controller receipt
for `physical_scene_archive`; then capture every sample using installed Chrono
mutable-face support, inspect resulting PNGs, and run the existing legacy replay
scene and PNG smoke. Author checks do not establish actual GPU rendering.
