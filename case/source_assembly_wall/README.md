# Connected source assembly execution

`Pilot.cpp` composes the original authenticated six-part input with the qualified
8 m/s, 5 micrometer gap, h=1/67108864 s integration experiment. Refinement 2 or 4
reduces h; step counts are actual intervals. Every source material, native M/J,
internal group and released external interface is retained. Deformation limits
match the qualified assembly smoke. The pilot adds no constitutive parameters.

`Execution.cpp` advances the existing case and records every committed interval,
with initial, cadence and final frames. Numerical rejection may close an explicit
accepted prefix. Any capture or file failure remains incomplete and returns 1;
a complete requested horizon returns 0 and a deliberate accepted prefix returns 2.
The archive writer owns all file/size/sequence checks. Neither module implements
mechanics, timestep adaptation or an alternative accepted-state owner.

Build this directory with explicit `ROBO_DYNA_TL_ROOT`, `Chrono_DIR` and CUDA
architecture. Run only under the workstation guard:

```
robo_dyna_source_assembly_wall INVENTORY WALL STEPS FRAME_EVERY NEW_DIR [REFINEMENT_1_2_4] [--stage-timing NEW_JSON] [--step-multiple 1_2_4_8] [--observe-force-stage] [--assembly six-part|seven-part]
```

At the base step, 1024 intervals span 15.26 microseconds. That is integration and
replay evidence, not a visible crash. Profile the measured cost and qualify the
native force-stage observation before promoting this experiment to a long impact.
Archive forecasting retains the inherited eight-segment logical-ledger limit;
the aggregate 1 GiB cap does not permit arbitrary interval counts.

Optional `--stage-timing NEW_JSON` enables fixed-storage host wall-time counters
around the existing case calls. The path must be outside `NEW_DIR`, checked
before startup, and the diagnostic report uses exclusive create without making
parent directories. A report failure is printed separately and preserves the
simulation's exit status and accepted archive. The report is written after
execution (also for an initialized failed/prefix run); it is not an accepted
artifact or part of the archive inventory. With no option, no stage clock is
read and no report is written. The CLI's existing progress/archive elapsed-time
clock remains unchanged.

`step_inclusive` measures initialized Step calls; the other named stages are
disjoint. `total` and `last_step` contain call/failure/valid-sample counts and
integer wall nanoseconds. This is host call time, including any existing waits,
not kernel time. No CUDA event, query or synchronization is added. Use an
identical request without timing to check accepted field/ledger parity before
using the measurements to select an optimization.

The optional `--step-multiple` requests `fixed_dt = (1/67108864 s) * multiple /
refinement`, with multiple 1, 2, 4 or 8 and the existing refinement 1, 2 or 4.
Both default to one. Named options may appear in either order after the optional
positional refinement; missing, unknown or repeated options fail before startup.
`PilotOptions` carries these execution choices into the existing pilot setup.
The exact resulting `fixed_dt` is already recorded in configuration output.

`--observe-force-stage` explicitly enables the case's existing acceleration
capture and source-partition kinetic observation before common publication.
It does not change the step or evaluate forces again. The observation refers to
the interval's force time, which differs from both carried half-step velocity
time and the newly accepted configuration time. It provides no global energy
acceptance threshold. Capture is disabled by default.

This changes the requested step only: native element, contact-rate, deformation,
source and resource checks remain in force. A multiplier's acceptance by the
parser does not establish coupled stability or response accuracy. Matched-horizon
runs must compare the admitted results before a larger step is used for a longer
impact. The options/parser host gate can be built independently of the solver:

```sh
cmake -S case/source_assembly_wall/options -B /tmp/robo-dyna-pilot-options -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/robo-dyna-pilot-options --parallel 1
ctest --test-dir /tmp/robo-dyna-pilot-options --output-on-failure
```

The optional `--native-rotation-domain` selects the explicitly qualified
source-native frame/normal guard while retaining the pilot's 1 rad geometric and
rigid-group limits. It preserves all actual quaternion output and per-step
rotation admission. See [the domain contract](../source_assembly_dynamics/NATIVE_ROTATION_DOMAIN.md)
for native evidence and the mechanically consequential shared-node spin caveat.

`--assembly seven-part` selects the separately authenticated seven-part inventory
and explicitly activates the original spotweld through the native TYPE25 import
policy. `PilotSource.cpp` owns that source/policy selection; an inventory from the
other scope fails before CUDA startup. Six-part remains the default. Both scopes
retain one owner, the same six rigid groups and explicitly released external
connections. Connector properties and combined endpoint M/J are archived under
the [optional TYPE25 schema](../../output/source_assembly/SOURCE_ASSEMBLY_TYPE25_SCHEMA.md).

The seven-part loaded qualification requests `--step-multiple 4` (h=2^-24 s).
Its 8h request fails the existing source-specific contact `h*sqrt(rate)` startup
guard, even though the six-part source admits 8h. No multiplier bypasses that
guard, and the default multiplier remains one. The admitted timestep and the
source's native element checks still govern every actual interval. The original
shell-only spin trace rejects connector endpoints; other supported probe nodes
retain the same diagnostic behavior.
