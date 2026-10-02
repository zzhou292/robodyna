# Native CAD adapters

`//src/integrations/cad:cad` compiles the four retained OpenCASCADE adapters;
`:irrlicht` adds the original mesh converter. These create mixed native bodies
and visual assets, so they explicitly depend on the current mechanics aggregate.
They are not an independent geometry-only domain.

The optional `@occt_sdk` provider admits the workspace source-built OCCT 7.9.3
installation, its pinned source and install manifest, public headers and all
40 actual DSO dependencies. It does not import a prebuilt Robodyna/Chrono solver.
Configure `ROBODYNA_OCCT_ROOT` to that qualified installation. Required license
and resource files remain in the admitted SDK.

Four original mains are exposed under `//examples/integrations/cad:native_demos`:
STEP import, robot assembly, profile extrusion and Irrlicht mesh conversion.
They retain their interactive runtime and data-path requirements. Building them
is distinct from running an unbounded GUI simulation.

`:native_test` performs real CAD mass/inertia evaluation and a STEP write/read
round trip without graphics. Provider, compile and runtime results are recorded
separately by the guarded integration build.
