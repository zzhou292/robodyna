# CW1 full-state analysis: prospective source contract

This source increment implements the spectral and switching portion of
`planning/QEPH_WALL_RECURRENCE_SCREEN.md`. It has not been compiled or executed
by its author. It consumes the existing immutable `WallRecurrenceModel` and
retained full native derivatives; it calls no native force routine, CUDA
operation, mechanics owner or trajectory runner. The native model and actual
point-law directional probes remain separate prerequisites. Passing this
screen alone admits no coupled impact trajectory, partial mask, T3, vehicle,
material-card equivalence, output archive or rendering.

## Owning modules and evidence

- `WallStateMetric` prepares D and applies its invertible similarity transform.
- `WallSwitchingSchedule` records the independent scalar witness and nine
  chronological event windows, starting at epoch one.
- `WallRecurrenceSpectrum` measures nonsymmetric constant-branch Schur spectra
  and endpoint-inclusive Gram spectra. `WallSequenceAnalysis` composes the
  qualified PowerGram blocks without a commutation assumption.
- `WallStateIdentities` checks complete observer rows, native negative cached
  force/couple kicks, allowed neutral directions and actual moving baselines.
- `WallRecurrenceAnalysis` assembles two constant branches and all nine
  schedules. `WallAnalysisComparison` records matrix/gain differences and the
  fixed coarse-step selection rule.

No existing recurrence helper, reduced-policy API, README, map, registration,
production or native source is changed by this increment. All 109/194 native
coordinates survive every branch, transform, spectrum and Gram. The legacy
dictionary's omitted-coordinate classification is used only to report the
largest historical `passive_feedback_*` entry and its indices. These
coordinates are not claimed physically passive under contact, and their
feedback is neither zeroed nor grounds for a block reduction.

Boolean builders stage output on failure. Analysis functions return records
that retain available numerical intermediates and a diagnostic when a screen
fails. In particular the raw scalar trace precedes window validation, the raw
Schur eigenvalues and residuals precede its verdict, and Gram antisymmetry is
measured before any symmetric eigensolve. An unresolved Gram preserves its
measured eigenvalues when available. The result records all eigenvalues,
leading Gram direction and controlling native coordinate. It does not promise
an eigenvalue interval certificate or a physical energy interpretation.

## Fixed metric and complete identities

Use the named model's actual maximum nominal frequency, not an averaged mass
rate or a fitted frequency. H=4096*2^-24 s, c=sqrt(200e9/7890), L=.02 m,
eta=H*c/L, wt=1/(1+eta), wn=max(wt,omega_c*L/c). D uses wn on world-X
positions, wt on Y/Z positions and every orientation tangent, and one on every
other coordinate. It is independent of h, amplitudes, boosts and observed
spectra. Finite positive weights and max(D)/min(D)<=1e8 are required.
The mathematical similarity seam admits 1..194 coordinates for independent
analytic tests; named branch analysis requires the model's exact full extent.

Observer rows compare x_next=x+h*v_plus and tangent_next=tangent+h*omega
in the actual dictionary scales. Independent cached-force and cached-couple
columns compare the negative h/native-m and h/native-total-J kicks; no freshly
produced cache enters that kick. Tangential common velocities, common world-X
rigid spin including its orbital velocity, and each node's source-X drilling
are checked against their complete kinematic outputs. Wall-supported normal
translations and tilts are not assumed neutral. All identity errors use the
frozen absolute normalized 5e-8 budget. No zero-feedback condition is imposed.

`CheckWallMovingBaseline` compares the actual retained output to h*V/position
scale and V/velocity scale in world X, zero in every remaining coordinate,
including all material, stabilization, cache and work slots. It never snaps
small native residuals to zero. The raw-collection coordinator must require
this check before interpreting each centered moving derivative.

## Scalar chronology and Gram counting

The scalar witness uses -gap, +8 m/s, omega_c and each of the six frozen h.
The first known-zero contact kick is h/2 and its drift is h; the initial-gap
inequality is checked explicitly. At epoch one the ordinary recurrence begins:
v_plus=v-h*omega_c*omega_c*max(x,0), x_next=x+h*v_plus. The diagnostic active
mask is x>=0, including either signed zero, matching the owning point law's
touching-or-penetrating convention; force is still zero at touching.

With total=H/h and N=total-1, entry and exit are **base epochs**, not endpoint
indices. A window (entry,exit) has counts (entry-1,exit-entry,total-exit).
Entry-major shifts {-1,0,+1}² produce exactly nine windows. Each must satisfy
1<=entry<exit<=total, and all counts sum to N. A shifted empty leading/trailing
inactive run is legal. Missing, repeated or out-of-horizon contact windows
reject; no observed event is fitted afterward. Maximum scalar depth is
reported as a witness, not an actual nonlinear shell depth admission.

For chronological a then b, use the retained helper's P=Pb*Pa and
G=Ga+Pa^T*Gb*Pa. Its G counts states before transitions. The final P_N^T*P_N
is added exactly once, giving N+1 states from epoch one through the horizon.
There is no doubled block junction. Continuously inactive and active branches
use the same ordinary count. The bounded mathematical seam permits up to
64 runs / 32768 transitions for independent noncommuting tests; the named
experiment uses only its three-run inactive/active/inactive sequence.

## Numerical decisions frozen before execution

Both raw and weighted constant operators retain full real-Schur evidence.
Residual and orthogonality must each be <=1e-10 and radius <=1+5e-8. Every
complex eigenvalue is retained; near-one and near-zero counts use 1e-6.
Similarity invariance does not excuse an unresolved numerical decomposition.
No spectral-radius test is imposed on a finite event product.

Raw and weighted Grams each require finite arithmetic, relative antisymmetry
<=1e-10 and a resolved symmetric spectrum (negative allowance
1e-10*max(1,Gram Frobenius norm)). Their complete eigenvalues and leading
direction remain in the record. **Only weighted** mean gain
sqrt(lambda_max/(N+1))<=64 controls absolute gain rejection. Raw gain is
diagnostic, including harmless large drift. The separately reported
sqrt(lambda_max) is the weaker individual-power bound and is not called a
uniform gain of 64.

Matrix comparisons use 5e-8*max(1,maxabs(both operators)). The raw shell probes
must also be compared by the caller, not inferred from a contact branch.
Consecutive-amplitude and cross-boost **weighted** mean gains use
.005*max(gains)+1e-10; raw gain differences are retained diagnostics and do
not create a second admission gate. Scalar comparison residuals are reported
upward and budgets downward with the owning q4_bounds
AbsoluteDifferenceUpper/MultiplyScalar/AddScalar primitives; the constants
are the represented binary64 values. Existing native/analytic 2e-12 budgets
are unchanged. No threshold is chosen from a measured matrix or gain.

The pure selector returns H0 only if aggregated points through 2H0 pass,
otherwise H0/2 only if all points through H0 pass, otherwise zero. The 4H0
point remains explicit evidence but cannot produce a larger selected step.
The caller must aggregate both fixtures, all three boosts, all amplitudes,
actual one-sided point-law probes, moving-baseline and startup prerequisites.
The selector does not authenticate supplied booleans or supply that evidence.

## Focused test and integration boundary

Thirteen new host functions are split into `WallMetricScheduleTest.cpp` (4),
`WallSpectrumTest.cpp` (5), and `WallIdentityComparisonTest.cpp` (4).
They reuse the named model preparation and existing analytic tolerance only;
they collect no full native Jacobians. Independent truths include a
long-double two-position scalar recurrence, closed 2x2 Gram eigenvalues,
the normal oscillator's modified quadratic, full Jordan/passive identities,
stable branches with an unstable alternating sequence, literal chronological
state sums, exact observer/cache signs, zero-gap masks, malformed-stage
preservation and failed comparison evidence. A deliberately enormous raw
Jordan gain passes its correctly scaled D norm; a wrong-sign spring and
wrong unit coefficient remain falsifiable controls.

Root registers these seven new numerical .cpp files and three test TUs with
the existing wall recurrence model/support, Eigen and GTest; C++17,
`-fno-fast-math -ffp-contract=off`. No new native library or solver target is
needed. Actual build, analytic execution, source-map closure, report byte caps
and guarded full-grid native decisions remain pending root integration.
Reports must preserve raw captures before decisions, <=32 MiB per file and
<=96 MiB total. These analysis objects do not bypass the report writer caps.
