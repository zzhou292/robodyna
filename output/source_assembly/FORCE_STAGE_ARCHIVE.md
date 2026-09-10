# Optional native force-stage observations

`Config.observe_force_stage` enables the existing case's optional accepted
`ForceStageSummary`. Configuration writes `observe_force_stage=true` only when
enabled. Disabled configuration and frame documents omit the extension; common
kinetic serialization retains the previous insertion order and field names.
Engine payload diagnostics may change when the engine's implementation grows;
this compatibility promise concerns the configuration and frame field schema.

Enabled frames contain `force_stage_kinetic=null` at epoch zero. At every saved
positive epoch the object contains the complete observation published by the
same common commit:

- `kind="native_force_stage_collocated"`, `owner_id`, `base_epoch`, `attempt`,
  `enclosing_epoch`, and `enclosing_time_s`;
- `source={source_instance_id,group_count,member_count}`;
- `phase={force_time_s,input_velocity_time_s,previous_frame_time_s,
  previous_drift_dt_s,kick_dt_s,drift_dt_s}`;
- `member_columns`, `ordinary_native_nodes`, `grouped_native_members`,
  `aggregate_columns`, `aggregate_groups`, `native_total_J`, `effective_total_J`,
  and signed `replacement_J`.

Both channel column strings and their exact array order are shared with the
existing midpoint/lagged-frame observations through `SourceAssemblyKineticSchema.h`.
The native total directly sums ordinary and grouped native nodes. The effective
total directly sums ordinary nodes and the aggregate rigid groups. Group primary
mass, inertia and principal corrections are already included in aggregate
channels; they must not be added a second time. Replacement is the stable sum
of per-group effective-minus-native differences, not a whole-assembly subtraction.

The observation belongs to the interval's **force stage at its base time**,
although it becomes visible with that interval's accepted endpoint. Its native
collocation uses pre-kick velocities and the actual captured accelerations with
the native `DT1/2` correction and updated group frame. The first stage declares
zero previous drift, half kick and one full drift. Later stages declare the
ordinary fixed-step durations. No additional final-endpoint force stage is
executed for output, and replay does not reconstruct this sample from the
differently phased accepted nodal velocities.

The reader authenticates the complete existing archive, then associates each
optional sample with its recorded CSV interval, accepted source descriptor,
owner, attempt, base/endpoint and all phase times. It checks finite values,
nonnegative kinetic partitions, native/physical/added and aggregate decomposition
residuals, direct native/effective sums, and signed group replacement. These are
persisted identity and arithmetic checks, not independent constrained trajectory
validation. Endpoint native internal/plastic work remains separate. This field
does not define a global energy balance or introduce an acceptance threshold.

The production writer obtains the pointer only from `accepted_force_stage()`
after live accepted capture. Disabled or initial pointers and wrong associations
reject before document publication. A rejected attempt cannot replace the
case's accepted pointer. Formatting-only host views are explicitly synthetic
test seams and cannot enter the production archive writer.

The additional object has fixed size. The complete 915-parent/1030-node host
format test expands every numeric token to its maximum reserved width and
checks the enabled frame against the unchanged 8 MiB reservation. Therefore
the shared forecast, 32 MiB file cap, 2 GiB archive cap and 1000-frame cap remain
unchanged. Serialization may allocate at output cadence; mechanics does not.

Owning host tests extend `robo_dyna_source_assembly_wall_output_check` with
optional declaration/null handling, exact disabled document compatibility,
full phase/channel mapping, reservation and invalid association cases. The
real-case writer test archives actual committed force-stage readbacks. Replay
tests reuse the original disabled archive plus an optional completed actual
enabled archive configured by
`ROBO_DYNA_SOURCE_ASSEMBLY_FORCE_STAGE_REPLAY_FIXTURE`; absent enabled fixtures
skip only those three tests. They stream every saved observation and reject
rehashed metadata, phase, partition, missing-record and declaration changes.
Host formatting tests are not evidence of CUDA recurrence or crash accuracy.

Author qualification: all 11 host formatter/sequence/archive tests pass with
freshly compiled changed objects and existing qualified native startup libraries
under one CPU and a 512 MiB virtual-address cap (373680 KiB peak child RSS).
The enabled worst-width frame is 6981691 bytes. XML and compiler output are in
`/tmp/force-stage-output-syntax-nilqfr_y/`. All 14 edited/new C++ translation units
pass host syntax against an isolated copy of the frozen live-case headers.
Root qualification now passes all six real-case CUDA writer functions
(`assembly-force-stage-output-tests-1`,8.94s), including committed-only capture
after a rejected attempt. The actual8h/128-step component pilot completes with
65 force-stage frames at15.258789us. The owning reader gate passes all14 assembly
functions, including the three enabled-archive cases and retained disabled
compatibility (`assembly-force-stage-replay-tests-1`). The9 base replay functions
also pass. These checks authenticate stored observations and do not establish
physical response accuracy or timestep convergence.
