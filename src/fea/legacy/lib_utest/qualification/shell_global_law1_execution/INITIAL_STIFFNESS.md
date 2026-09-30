# Initial global LAW1 stiffness uses its immutable profile

C1BUF3 lines56/61 initializes reported THK from declared THKE. In CZFORC3,
the complete CNCOEF3B call at467 consumes GBUF%THK and THKE with ITHK before
the material update and CNDT3 at656. CNCOEF3B's selected ITHK1/ISMDISP0 branch
sets THK0=MAX(EM20,THK); it has no time-zero exemption. CNDT3 then uses that
effective thickness/volume for its translational and rotational coefficients.
The exact source files were git-blob checked against a62b and are pinned in
`initial-stiffness-source.json`.

Consequently the virgin global LAW1 profile uses the initial reported thickness
(equal to reference thickness) through the same qualified coefficient selector
as later forces. QEPH converts native EM20 with coefficient_working_length_m;
T3's C3COEF3 takes reported thickness directly and has no corresponding floor.
The projection working metric is a separate descriptor. No coefficient result
is inferred from a material point, no startup history is advanced and initial
CINMAS is not substituted for runtime STI.

The immutable catalog supplies the profile to mapped startup validation. The
same optional private profile span reaches initial assembly through the existing
mixed assembly adapter. Legacy/non-global roles retain the null/default route.
Global role with a missing/invalid profile fails instead of silently selecting
reference thickness. Later accepted stiffness still comes from the actual force
cache. Exact profile fields remain part of catalog/failure identity and optional
storage forecasts.

The native boundary test initializes original native and TL references with
virgin thickness below/on/above the floor, in metre and millimetre contexts,
then compares the complete native force-stage STI diagnostics and native nodal
scatter factors against mapped initial packets. Stationary prescribed forces
supply the independent existing CNCOEF3B/CNDT3 and C3COEF3/C3DT3 paths; they do
not create a physical step or native restart. Coupled owner tests separately
exercise actual initial assembly, accepted caches and common publication.
