# Accepted V5 limiter report

Requires companion TL commit `994658e` (`cin-limiter-diagnostic`). Defaults keep diagnostic
capture disabled. Set `Config.structural.capture_limiter=true` on the actual
loaded dynamics. Trial copies the successful owner receipt through the private
owner bridge; the existing candidate/accepted observation swap publishes it.
No extra CUDA readback, phase, clock or archive schema is introduced.

After acceptance, `StructuralLimiterReport(dynamics, byte_cap)` uses only that
actual dynamics' retained wall/source setup and its last accepted observation.
It emits actual physical NID/group identity, winner coefficients and factor,
ordinary constraint flags or rigid trace/principal inertia, complete incident
prepared element IDs and direct CIN donors. Shell THK/rho/E are explicitly
**reference SI inputs**, and incidence does not pretend to partition current
summed stiffness. Wall and TYPE45 contributions remain unseparated. A rigid first
member identifies the body; it is not singled out as the cause of the trace.

Memory: StepObservation adds the fixed public receipt in each of two existing
slots; existing `sizeof(Storage)` forecasts include it and the appended policy
flag. No per-node array or per-step allocation is added. On-demand report storage
is bounded by `2*byte_cap + (2048+CIN row count)*sizeof(size_t) + 4096`, with
byte_cap at most 8 MiB and CIN rows at most the existing 65536 limit. A count pass
preflights the complete JSON before returning any report; cap failure cannot
truncate a source inventory or change the accepted owner.

Root reuses the active `vehicle_run` original CMake cache and normal source env:

```sh
cmake --build <existing-vehicle-run-cache> --parallel 1 --target robo_dyna_vehicle_run_original_check
ctest --test-dir <existing-vehicle-run-cache> --output-on-failure -R '^(vehicle_limiter_values|vehicle_run_supports_limiter)$'
```

If reconfiguration is necessary, preserve the original cache's source paths,
`ROBO_DYNA_VEHICLE_RUN_LIVE/ORIGINAL=ON`, TL checkout and
`Chrono_DIR=.../chrono-vsg-r1/lib/cmake/Chrono`. Root owns actual execution.
`vehicle_run_supports_limiter` uses all 376930 physical nodes, 4980 solids,
142 structural beams and 44 joints. It sets the **unchanged baseline** dt2e-7,
wall stiffness1e10 N/m^3, gap1e-6 m and penetration limit.002 m. It tests discarded
first attempt, retry, two acceptances, exact report cap and repeated read.
Stdout has two `CIN_STRUCTURAL_LIMITER {...}` lines; GTest XML records
`first_limiter_json` and `second_limiter_json`. These identify the actual limit;
they do not authorize a larger step or establish long-run stability.

Normal V5 initial/loaded-prefix and archive/capture regressions remain required.
The source author performed no build, native, GPU or complete-source execution.

Source-only identity: `python3 -B case/vehicle_dynamics/limiter/verify_sources.py`.
