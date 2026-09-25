# QEPH projection working-unit qualification and design

Status: native captured replay qualified at fe30c7ce (28 host tests plus source
check). Immutable reference-level working-length adapter is source-ready and
awaits its own host/CUDA and unchanged coupled trajectory gate. The source branch starts from runtime pin
bccc7801d28a2666335db4d6ae76b0e204195c1c, retaining existing native QEPH oracle and production source identities.

## Read-only source finding

Native CZCORC1 forms centered local coordinates by subtracting physical X and
rotating by its orthonormal frame; it never divides those lengths by element
size. COREL/X13/Y13/Z1 remain native working lengths. It rotates native VR
without a length factor, corrects translational velocities, calls CZCORP5 and
only afterward applies AREA_I rate normalization. TL CurrentGeometry,
GatherRates, CorrectMidpointVelocity, ProjectWarpedRates and NormalizeRates use
the same order on owner SI fields, with no intermediate native-length adapter.

Full projection forms D=A(length squared)+C, where C includes4 minus products
of unit normals, and AR contains both x times velocity and angular velocity.
This mixed numerical metric depends on the working length unit. Under length
scaling r, its two parts become r squared A+C and r squared a+b (for a fixed time
unit), so simply converting a millimetre model to SI does not establish full
warped-branch equivalence. The flat branch bypasses this metric. This is a
source-grounded explanation to test, not yet the proven cause or size of the
observed coupled drift. No tolerance change, flattening or source-force clamp
is justified by it.

Exact source locations: native czcorc.F198–243,337–357,424–439,576–610;
czcorp5.F202–251; production QephCurrentFrame.h40–64,
QephProjection.h26–85, QephKinematics.h59–65 and ShellBatchFields.h13–20.
The native donors are pinned under qualification/native/qeph and workspace
crash-work/deps/openradioss-shell-execution-trace. Their unit-independent frame
rotations must not be confused with normalization of coordinate magnitudes.

Property Idrill2 is already correctly mapped to routine IDRIL0:
HM_READ_PROP01 reads Idrill into ISROT and maps2 to0 (145,214–216); CGRTAILS
reads IGEO20 and publishes IPARG41 (554,959); CZFORC3 reads that field at300
and passes it as CZCORC1's IDRIL at453, then CZCORP5 at583. The matching force
projection receives the same flag at CZFORC3:764. Changing drilling mode would
be a different algorithm and is not the proposed correction.

## First executable qualification slice

Reuse `lib_utest/qualification/native/qeph` and its existing mechanically
extracted CZCORC/CZCORP5/CZPROJN, native bridge, geometry and force assertions.
Do not copy or translate a second Fortran core. Add a small standalone owning
qualification target in this directory only after the actual packet is sealed.
The test-only packet loader must retain exact binary64 inputs, native caller
controls, temporal phase and defined-channel masks; it cannot synthesize missing
VR, prior history or controls from an accepted animation frame.

Replay two levels separately:

1. Raw CZCORP5 entry/exit and matching CZPROJN operands establish the full
   projection operator on identical recorded geometry and rates/forces. Compare
   native metre and millimetre working spaces after explicit dimensional output
   conversion. Keep all branches/controls and operation ordering unchanged.
2. Complete CZCORC1 input X/V/VR and actual DT1/DT12 establish the caller chain,
   including midpoint correction and planar switch. Reuse the existing native
   prescribed oracle where its control contract matches; otherwise add a small
   qualification-only wrapper around the same retained routines with captured
   controls. NPT0/3 both differ from the special NPT1 bypass, but that source
   observation does not license changing a captured control silently.

The packet replay reports original native units and SI-converted observations,
per-field scales and tolerances. It must distinguish same-input port/native
agreement from same-physical-state unit conversion. The existing QEPH scale
sweep establishes the former; it does not compare one physical state expressed
in different working units.

Required cases/gates:

- Actual warped impact packet and its neighboring planar state; asymmetric,
  nonsingular warped synthetic control with nonzero translational and angular
  rates. No conclusion based solely on a rigid or zero-force special case.
- Native metre versus millimetre working-space replay, and at least one other
  declared length scale. Preserve time units and every dimensional conversion.
- Rate/force transpose and virtual-power agreement with independent force and
  velocity seeds, including angular components. Reconstruct from the actual
  native packed layout; do not mistake v13/v24/vhi for four node velocities.
- Planar bypass and legacy one-metre behavior remain exact on all old fields;
  the old default profile is not silently changed.
- Reference/history binding includes the chosen scale. Reject zero, negative,
  NaN/infinite and unrepresentable scale conversions before any publication;
  stale or mixed scale references must not match cached histories.
- Actual host/CUDA parity and whole coupled accepted-step comparison only after
  the reviewed source change exists; matched end-to-end timing remains separate.

## Proposed production seam, pending packet and placement review

Prefer an immutable reference-level projection working-length descriptor,
expressed as metres per native projection unit, default1m for legacy behavior.
The source factory derives it from its authenticated native unit declaration.
It is not a per-step knob or a geometry-derived regularization parameter.
Reference identity, matching/history checks, source descriptor conversion,
resident copies and sizeof-based resource forecasts must carry it consistently.

A complete dimensional derivation must cover BOTH rate projection and force
projection, including D/DI/DB, AD/AR and their reconstructions. Replacing the
constant4 or changing only D is insufficient. A native-coordinate boundary
around the relevant stages may be clearer than algebraic rescaling; either
choice needs exact same-input native replay and a special identity path for1m.
The interface decision waits for root/lead review of the captured packet and
inventory placement. No production code is changed in this checkpoint.

Dimensioned thresholds elsewhere in the source, such as the EM10 local-length
comparisons before projection, also need their declared working-unit contract
recorded. This slice must not claim that all structural dimensional constants
were globally closed merely because the warped metric is understood.

## Implemented boundary awaiting qualification

ReferenceInput::projection_working_length_m declares metres per numerical
projection unit; default1 takes the unchanged raw rate/force paths. Startup,
history and binding identity authenticate it. Pair/collection/QBAT inventory
versions are6/7/8 with52 pair words,29 words per QEPH,23 perT3 and47 perQBAT.
QBAT rejects nondefault projection length because its distinct force formulation
does not consume this QEPH projection. Retained native mass/inertia are unchanged.

The local adapter converts only projection operands: x and v divide byL,
area-like terms divide byL², angular rates remain unchanged; projected v returns
by multiplying L. Force staysN, local moments divide byL and projected couples
return by multiplying L. The original leaves are unchanged. Kinematics physical
fields remain SI. DI/DB are exact algorithmic coefficients in the tagged
ProjectionMetric, not incorrectly advertised as SI tensors or rounded via a
second representation. Force projection rejects a metric inconsistent with its
immutable reference. Invalid conversion, overflow or nonzero-to-zero underflow
rejects before staged output publication. Ordinary material/history calculations
remain SI. Other native dimensioned thresholds are outside this boundary.

Tests cover captured planar/warped profiles atL.001/.01/1, full rate/force duality,
legacy exact fields, invalid and mismatched descriptors, both-direction public
force/history rejection, device execution across launch sizes, and affected
shell inventory/placement regressions. Physical qualification uses the original
1000-step moving-scene acceptance tolerances; no gate is relaxed by this change.
