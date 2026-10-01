# Bounded live Chrono demonstrations

This small support library observes the original examples' physical systems.
It never constructs a model or advances dynamics. Each example retains its native
setup, solver and timestep and passes every completed step to `CaptureSession`.
The normal interactive CMake path stays behind the unchanged preprocessor branch.

The opt-in executable takes `--output`, `--steps`, `--capture-every`,
`--chrono-data`, and optional `--headless`. Steps must be a multiple of the sample
cadence. Captures include step 0 and the final step: spring and SCM defaults give
151 images, while the NSC collision example gives 101. The demo source is a
declared runfile; its identity is recorded alongside the observed physical clock.

`CaptureVisual` disables interactive camera motion. Configure it before
initialization to freeze one loading worker and disable render-rate skipping.
The shared `viewer/VsgImageCapture` primitive renders twice without advancing
physics because Chrono exports the preceding swapchain image. The recorder checks
the real step counter and clock before and after capture, decodes each PNG, checks
dimensions, and hashes the file. It uses the existing ArtifactIO utilities.

Successful image capture publishes `robodyna.chrono_live_capture.v1`, with
`simulation_executed_by_viewer=true`, real step/time rows and model telemetry in
`run-summary.json`. It does not manufacture accepted-archive receipts or claim a
restart. Headless runs produce only the run summary. A failed or interrupted run
keeps partial artifacts and cannot publish a complete capture manifest.

The current examples explicitly use CPU dynamics and Vulkan graphics; GPU graphics
is not GPU physics. Model-specific trackers report motion/contact/soil changes and
backend counters independently. Existing archive replay schemas remain separate.

Headless mode forbids any attached visual system, even an uninitialized one:
Chrono's dynamics setup can initialize attached renderers automatically. The
recorder checks this at state 0 before the first physical step and again at every
observation and completion. The host regression uses a CPU-only visual test double;
it never constructs or initializes VSG.

PNG output has a 6 GiB cap, with 32 MiB per image and 4 MiB reserved for metadata.
These disk limits do not replace workstation RAM, GPU-memory or CPU guards. The
root launcher remains responsible for bounded execution and cleanup.

`//examples/support:capture_values_test` exercises count/cap boundaries, clock
rounding, real headless stepping, create-only output and incomplete-run rejection.
Graphics qualification additionally requires a short real capture, same-state
render checks, full encode/decode verification and first/middle/final inspection.
