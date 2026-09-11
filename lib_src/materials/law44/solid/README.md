# Selected solid LAW44 point

This module implements the tabulated isotropic SIGEPS44 branch used by original
rear metals PID2000016/2000392: IPLA1, VFLAG2, filtered total strain rate,
YSCALE1, CA0, ICC1 and IEOS0. It preserves the native finite default limits,
forward interpolation cursor, six stress and six engineering-strain histories,
plastic strain and filtered rate. Inputs and outputs are SI. `WorkingUnits`
records whether native finite stress limits/floors originated in SI or t/mm/s.
It does not change the point's SI elastic or rate equations.

`Prepare` validates a borrowed immutable 2..1024-point monotone curve and all
scalar parameters. The caller must retain that backing unchanged. `Update`
uses a strictly positive supplied timestep and the actual MMAIN AMU measure;
it stages the result before publication. Rates/stresses must already be in the
same current material frame. The caller owns the physical frame/geometry and
the accepted transaction. No current density is reconstructed from strain.

At zero accepted plastic strain native hardening is E, so the first plastic
increment has ET=.5. Later hardening uses the retained curve slope and rate
factor. The returned sound speed uses the solid bulk/shear expression, not
the reader's shell-style PM27 estimate. Yield caps and strain checks follow
SIGEPS44's own default branches, including its four-step Newton approximation;
they do not authorize parent removal or a generic failure policy.

Energy/viscosity integration, constructor TT0, alternative hardening/rate/EOS
policies and element/source/owner admission are outside this increment. The
future rear-metal element is the native eight-slot H8 branch, including 109
records with repeated positive source NIDs, and needs separate ICPRE1 gates.

Qualification: `lib_utest/qualification/solid_law44_point`. The only shared
production change extracts the existing VINTER linear segment expression into
`VinterSegmentValue`; the existing binary segment selection and arithmetic
remain unchanged. LAW36 regression tests exercise that path.
