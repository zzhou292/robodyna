This host-only analyzer replays recorded one-step QEPH transitions against the
independent native layered reference. It uses the original authenticated six-part
inventory, existing source shell/material adapters, and the completed optional
spin trace. It creates no solver owner, changes no physical state and performs no
CUDA operation.

Build owner: this directory's CMakeLists, with an explicit ROBO_DYNA_TL_ROOT that
contains the qualified `shell_layered_native_reference`, the installed Chrono
package and local GNU Fortran compiler. The executable is
`robo_dyna_source_assembly_spin_analyze INVENTORY COMPLETED_TRACE NEW_REPORT_JSON`.
The test target is `robo_dyna_source_assembly_spin_check`; set
ROBO_DYNA_SOURCE_ASSEMBLY_INVENTORY and optionally the completed actual
ROBO_DYNA_SOURCE_ASSEMBLY_SPIN_TRACE fixture.

SourceContext authenticates the pinned inventory and uses the existing immutable
source input adapters, including the explicit direct-import material rate policy.
It independently initializes native references from original geometry/material.
PacketInput checks every parent/source/local-node identity and the distinct base
and enclosing phases. Analysis validates header bounds, full stamp/group scope,
ordered attempts/cadence, final-prefix/footer counts and exact byte subtotal.
The trace is read once under its 256 MiB cap and each JSON row is parsed separately
under 64 KiB, with duplicate-key rejection. No incomplete trace produces a report.
Input SHA256 and byte count identify the exact bytes analyzed. These are prescribed
qualification inputs; the parser does not manufacture a live-owner authentication
or a solver restart authority from a JSON file.

Each sampled native evaluation receives the recorded accepted native history and
three point states, plus the actual next endpoint x and carried midpoint v/omega.
The result is compared to the separately recorded candidate. Metrics retain units,
maximum/RMS difference and the worst source parent. HOURG and rate entries with
different native units are split. The existing pilot metric implementation has a
small owning target for reuse; its arithmetic is unchanged.

LocalDiagnostic also removes only the selected parent's own normal component of
the prescribed spin, holding x/v/other spins and old history fixed, and evaluates
the same native routine again. This frozen-state sensitivity is compared against
the unmodified native result, separately from port/native agreement. It is not an
alternate trajectory and is not a common-node null-mode certificate: two incident
warped parents have different normals. Actual tangential torque, native stiffness,
plastic response and inertia remain in the simulation. Carried power and per-node
native/added-J carried kinetic values are explicitly staggered diagnostics, not
collocated work or aggregate rigid-group energy. Sampled work increments must not
be summed over gaps to imply a complete work integral.

The tool reports mismatches; it introduces no numerical acceptance threshold,
energy tolerance or convergence verdict. The JSON report is capped at 32 MiB and
written create-only using ArtifactIO's externally serialized writer contract.
Tests cover a supplied pair with intentional point/couple differences, wrong
source/phase/origin, missing/truncated/duplicated trace rows and forecast scope.
An optional complete actual trace traverses every paired native evaluation.

Author gate: all four tests pass against the completed 539-stage trace in 337 ms;
source replay plus report generation peaks at 34,708 KiB with one CPU/512 MiB AS.
Native libraries are the already-qualified frozen 804eae9 donor composition; no
Fortran or CUDA build was run by the analyzer author. Actual evidence is
`crash-work/reports/source-assembly-spin-native-analysis-2.json`, from
`source-assembly-spin-prefix-1.jsonl` SHA256
`c579d677004cff35308ecbc0e1bb9ab6c962352d0853646dcccc5ab07e3fd383`.
The first report is retained; report 2 adds explicitly phased nodal carried K and
forms small work differences per native channel before summing.
