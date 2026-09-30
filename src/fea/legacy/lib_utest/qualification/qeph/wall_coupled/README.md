# CW0: short QEPH and finite-wall transaction qualification

2026-09-09. **Source and independent review frozen; not executed.** The operands
and budgets below were frozen before runtime implementation. This is a short
coupled algebra/publication gate, not an impact or stability admission.
No production mechanics, owner, capacities or timestep policy changes.

## Frozen experiment

Use the existing BQ3 one/two-cell connectivity, 4/6 all-free nodes, 20 mm square
Q4s, E=200e9 Pa, nu=.3, rho=7890 kg/m3, t=.001648 m, native physical plus
area-added isotropic inertia, and unchanged native DM/DN=.015. Material/history
and the accepted internal force cache start at zero; velocities and spins are
zero; initial quaternions are identity. The contact preload is initial energy.

For each normalized BQ3 material coordinate (u0,v0), evaluate binary64 operations
in this order: u=.02*(u0-.5*cell_count), v=.02*v0, s=1/128,
c=sqrt(1-s*s), X=2^-12+s*u, Y=c*u, Z=v. This is a common tilted **reference**
plane, not a displaced flat reference. Use these same represented coordinates
for native/QEPH startup and independent contact area preparation. The decimal
depth ranges are approximate: one cell .166–.322 mm; two cells .088–.400 mm.
No snapping, imposed prestrain or decimal-exact plane/area claim.

Finite wall: existing Square() two triangles at X=0, Y/Z corners +/-2 m,
unchanged face IDs; admitted projected motion box Y/Z +/- .05 m and clearance
1e-6 m. Frictionless reference-area-lumped-nodal-wall-v1, kappa=4e5 N/m3,
maximum depth=.0005 m. Per-parent force and energy certificates retain exactly
5e-7 N and 1.2500000000000005e-12 J. There are no other applied loads.

Run h=H0 and H0/2, H0=2^-24 s, through the same 4H0 horizon: 4 and 8 intervals.
First kick h/2; subsequent kicks h. Qualification=0x4357305052454631,
shell configuration=0x4357305348454c31, contact configuration=0x43573057414c4c31,
wall binding=0x43573046494e4931. The identity admits these short checks only.

## Frozen comparisons and failure gates

Reuse the BQ3 native scalar recurrence, complete Q3c force/history comparisons,
owner tolerances (x 2e-12*.02 m; v 2e-12*sqrt(E/rho)*1e-4 m/s; omega v/.02;
q 2e-12), and native internal balance checks unchanged. BQ3's physical ledger
floors remain its declared bending-load scales even though CW0 applies no such
load. Work/momentum arithmetic remains 256*epsilon*sum(abs(operands)) plus
1e-12 times that fixed physical scale; it is not a response/error tolerance.
Contact-specific ledger floors: energy kappa*(count*.02^2)*(2^-12)^2,
impulse kappa*(count*.02^2)*2^-12*(4H0), angular impulse .02 times that
impulse. Include reported contact force/potential/work/impulse uncertainty.

Every step checks independent native shared mass/J and accepted-cache scatter,
actual host nodal law versus device nodes/parents/certificates, complete material
and source-work aggregates, kinetic kick work, linear/angular momentum with
actual rounded drift, contact kick/drift work, potential defect and wall impulse.
The convex contact defect is enclosed in [0, .5 sum(k_i*dx_i^2)]. EINT/EVIS stay
native work, not an elastic-potential identity. Both participant outputs and
native proposals are staged before the existing single joint commit.

Require a noninitial omitted-contact and omitted-internal-cache kick control
each exceeding 32 existing owner velocity/spin budget units; record maxima and
their epochs. Record endpoint-cache-too-early control separately. If a required
control is too weak, retain the failed gate; do not silently change h/preload,
stiffness, source material or budgets. Require nonzero produced internal cache.

Failure/retry checks cover contact-success/shell-failure and the reverse order,
stale/foreign identities, rejected precommit result readback and bad receipt.
Every failure preserves accepted owner/history/cache and previously published
contact output; clean retry has identical candidate numerics before committing.
No receipt follows any failed numerical/ledger assertion. No CUDA fault injection
or recovery claim is needed for this composition gate.

## Build and evidence boundary

Only this opt-in subdirectory is new. It reuses qeph_coupled_fixture and
tl_nodal_wall_contact_device; root owns parent registration and serialized runs.
The parent opt-in flag is TL_QEPH_ENABLE_WALL_COUPLED, requiring COUPLED/BATCH/CUDA.
Target: qeph_wall_coupled_check; four CUDA GTest functions, no skips. Capture
`--gtest_output=xml:NEW.xml` with the guarded process report. Successful tests
exercise 64 native cell intervals; the seven rejection variants each use a fresh
two-cell owner with at most two accepted intervals.
One process, one CPU, 1 GiB RSS, 120 s timeout, 4 GiB device-growth guard;
device-wide growth is not reported as owned bytes. Participant allocations are
captured before intervals and checked unchanged; no per-step device allocation.
This uses small fixed host staging arrays; existing native/GTest diagnostics may
allocate transient host strings. The source-map pins current parent/shared
fixture wiring and local sources, delegating donor originals to the existing
verified native manifest; it is not a hermetic compiler/runtime closure.
Independent review found and fixed only a nested row-attempt retag in the
clean/retry comparison before execution; no numerical budget was changed.
