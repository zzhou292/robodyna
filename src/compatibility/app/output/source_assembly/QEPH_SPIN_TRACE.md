The optional source-node QEPH spin probe records actual retained native packets
and the loads already copied by the assembly case. It changes no force law,
rotation guard, timestep, owner allocation, CUDA call or state clock.

Enable it on the existing pilot with:

```
--spin-node 2181592 --spin-output NEW_COMPANION.jsonl --spin-every 8
```

Both node and output are required. The companion must be new, outside the
accepted archive and distinct from the optional stage timing output. The normal
accepted archive schema is unchanged. The case option defaults to zero/off;
when enabled it preallocates two `QephSpinObservation` slots, charged to the
existing host budget. Each slot supports at most eight complete incident QEPH
parents. Group members, any incident T3, missing nodes or overflow are rejected.

The actual six-part source node 2181592 is global 459 with exactly two incident
parents: source 2214871/2214872, source parent 445/446, QEPH 392/393, local 1/0,
PID/MID/SEC 2000145, curve 2100180. Both are warped ELFORM16 parents. These IDs
are discovered and crosschecked against authenticated binding connectivity;
the production observation does not hardcode their indices.

Each emitted `accepted_force_stage` row contains:

- Full accepted base and enclosing committed stamps, attempt and source instance.
- Source/global node identity and authoritative native mass, total/physical/added J.
- Base endpoint x/q, carried midpoint v/omega, actual assembled force/couple,
  negative complete native internal parent-couple sum and residual.
- `parents`: complete source IDs/connectivity, source-ordered nodal x/v/omega,
  actual positive internal forces/couples, FOR/FOR_G/MOM/HOURG/STRA/thickness,
  internal and viscous work, full native rate/frame/projection packet, native
  diagnostics, all three point stresses/PLA/rate filters and plastic work.
- `enclosing_candidate_parents`: the same fields from the actual enclosing
  candidate and its motion. Before the commit these remain private. The paired
  old history/points plus enclosing prescribed motion form an exact one-step
  native replay input even when trace cadence exceeds one interval.

The native rate packet retained at base epoch n was evaluated at x_n with
carried v/omega_(n-1/2), over the preceding native material interval. Its origin
is `base_stamp.reaction_time`; its native history endpoint is `base_stamp.time`.
The enclosing packet has origin `base_stamp.time`, endpoint n+1. Initial base
forces/history have never evaluated native kinematics: `available=false` is
explicit, while the enclosing epoch 1 packet is evaluated.

Normal/tangent projections use each native local nodal normal mapped by its
retained native frame. Powers are products of a retained force and carried
midpoint spin. They are diagnostics, not collocated work, dissipation, or an
energy acceptance certificate. Native HOURG/STRA/rate arrays retain their owning
mixed-unit conventions; point stress components remain native corotational.
No finite-difference spin is substituted for an original force.

The pure observer checks source/phase association and preserves outputs on
failure; the case authenticates live readbacks and publishes only after its sole
common commit. The optional observation has no authority to commit or advance.
Rejected attempts leave the previous observation intact. Prepared packet source,
configuration, qualification, attempt, epoch and complete time fields are checked
against the actual prepared view before observation.

The writer must receive every successful interval in order, even if not saved.
It saves epoch 1, cadence points and an unsaved accepted terminal before writing
its final completion footer. A rejected run can finish an explicit accepted
prefix, including epoch 0. Duplicated/skipped intervals or I/O errors poison the
trace; missing footer means incomplete. Creation uses O_EXCL/O_NOFOLLOW. Each
row is bounded to 64 KiB, total forecast to 256 MiB; planning reserves all rows before
startup. For 8192 requested steps/cadence 8 the conservative forecast is 67,371,008 B.
Archive and companion completion are separate: a complete companion does not
claim its sibling archive also completed if a later archive I/O operation fails.

Owning qualification targets:

- `case/source_assembly_observation`: `robo_dyna_source_assembly_observation_check`
  and `robo_dyna_source_assembly_spin_fields_check` (host).
- `case/source_assembly_wall/options`: `robo_dyna_source_assembly_pilot_options_check`
  (existing host CLI target; named flags are included).
- `case/source_assembly_dynamics`: `robo_dyna_source_assembly_dynamics_check`, filter
  `*OptionalSpin*` (actual 64-step contact, default/off parity, exact host charge,
  unchanged device allocation, source packets, late rejection and exact retry).
- `output/source_assembly/wall_artifacts`:
  `robo_dyna_source_assembly_wall_artifacts_check`, filter `QephSpinTraceLive.*`
  (complete, rejected/zero prefix, ordering/poison and create-only/cap tests).
- `case/source_assembly_wall`: `robo_dyna_source_assembly_wall` (same pilot CLI).

Author qualification: five observer/serialization tests and eight CLI tests pass
under 1 CPU/512 MiB; all new case/CLI/test C++ sources and Step.cu host syntax pass.
No CUDA runtime or numerical qualification is claimed by those host checks.
The root schedules actual GPU gates and the source trace before any decision to
replace the total-q guard with a narrower qualified geometry/director domain.
