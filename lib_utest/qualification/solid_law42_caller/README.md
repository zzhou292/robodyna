# Shared active LAW42 solid caller values

This is a value helper for the explicitly selected HEPH/S6Z total-strain10
composition: LAW42 alpha2, one Ogden term, no Prony, ISELECT0, NPG1,
Lagrangian volume update, default QA1.1/QB.05 and active material only.
It owns no mesh, accepted selector, clock, source admission or completed run.
A tensile cutoff returns an error without replacing the output. The mapped
original cutoff remains 1e26 Pa; the point leaf still implements its native
cutoff separately.

`CallerHistory` holds the single aliased native material/global energy density,
including prior physical hourglass work. The caller returns the material-phase
energy and signed work increment; a family then adds both native stabilization
work halves to that same history. `TotalStrain` retains the nonsymmetric
material gradient and native engineering off-diagonal convention.

`PhysicalHourglassModes` contains only the common eight-slot physical mode
rates, ICP1 coupling and force scatter. A wedge caller must supply its qualified
six-to-eight expansion, own geometry and final fold. `SolidCharacteristicLength`
is the engine six-face LAW42 SDLEN3 branch with IDT1SOL0/IDTS6=0; it is distinct
from the S6Z startup five-face rule.

Dimensional values and packet floors use SI, matching the previously qualified
LAW36 caller convention. This does not claim original working-mm behavior for
sub-1e-20 m3 elements. Native source revision is
`a62b27e6baa555d222a580d6218867d0be4d70b5`: SRHO3, MMAIN, MULAW,
MQVISCB, SZHOUR3/S6ZHOUR3 and SDLEN3. The complete independent native caller
qualification is owned by the subsequent HEPH/S6Z family gates; the tests in
this directory establish host contracts only.

Configure this directory and build/test `law42_caller_host_check`. Bazel owns
`//lib_utest/qualification/solid_law42_caller:host_check`.
