# Bounded recordings of the original MBD examples

These targets compile the retained Chrono examples through Robodyna's native
Bazel graph. The original physical setup remains in its original source file.
`ROBODYNA_CAPTURE_DEMO` adds explicit bounded operation and the common capture
session; builds without that definition retain the interactive example loop.
No new force law, contact method or integrator is introduced.

| Target | Original setup | Bounded delivery |
| --- | --- | --- |
| `//examples/mbd:spring` | Two 1 kg bodies; original built-in and callback spring-dampers, k=50 N/m, c=1 Ns/m, rest length=1.5 m, zero gravity | 6,000 original 0.001 s steps; capture every 40 steps: 6 s and 151 images |
| `//examples/mbd:collision_nsc` | All 87 original falling bodies, five fixed walls and rotating mixer; Bullet NSC, PSOR with 50 iterations | 2,000 original 0.003 s steps; capture every 20 steps: 6 s and 101 images |

The collision realization explicitly seeds the original `ChRandom` generator
with `20260930`. Density, geometry, materials, initial-position distribution,
motor speed and solver choices are unchanged. Its seed is reported; this does
not promise bitwise agreement across different standard-library/platform builds.

The examples execute **retained CPU dynamics and GPU Vulkan visualization**.
They demonstrate functional retention through the unified build, not a newly
qualified CUDA multibody solver or independent FEA/MBD library closure.

## Diagnostics

`Telemetry.*` only reads existing state after each original dynamics step.
Spring observations include finite state, actual displacement/velocity, original
native-versus-callback agreement, and error against the continuous damped-oscillator
solution. The reference never supplies body motion or forces. Numerical error is
reported explicitly; rendering alone does not qualify accuracy.

Collision observations retain the exact 93-body inventory and record finite
position/orientation/velocity, maximum displacement and speed, contact counts and
the chosen seed. These are bounded scalar observations, not a complete energy
or contact-work ledger.

The common `examples/support` API owns command-line admission, actual step/time
checks, explicit visualization data, screenshot cadence and final receipts.
Each physics step is called once and must report success. Two renders used to
capture a PNG see the same state and never advance dynamics. Headless mode skips
visual initialization and produces no fabricated image-capture receipt.

## Qualification and operation

Build and run only through the existing bounded workstation policy. Begin with
headless short/full-horizon runs and inspect telemetry. The spring must show
nonzero motion and close native/callback agreement; its analytical error remains
measured. The collision case must retain all bodies, observe contacts and show
finite motion. Capture the declared fixed cadence, then use the common video
encoder and full decode checks. Review first/middle/final images before declaring
the visual delivery complete.

The source baseline and provenance are recorded in `PROVENANCE.json`. Captures
record the actual adapted source digest. None of the original preserved worktrees
or earlier Yaris artifacts are changed by these targets.
