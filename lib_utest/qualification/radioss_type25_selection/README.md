# Native TYPE25 selection stages

Current source checkpoint implements the complete selected **retained**
COR3_1 → DST3_1 → GLOB_1 packet. Continuation/new-impact classification,
winner folding and sliding/inventory staging remain subsequent stages.
No physical contact owner, second clock or keyword parser is introduced.

The retained input supplies authentic current source geometry, native normal slots,
gaps, both side coefficients, row identity/generation and a prior native row.
Classification uses main*abs(secondary); this is NOT the min/clamped force K.
An active retained row requires a valid old sector and matching global/local main
and processor. Zero old sector is an undefined native input, not repaired to1.
The explicit profile is local IGAP1/INACTI5, no thermal or applied-gap extension.

NativeContactRow.selection_metric replaces the misleading time_s field without
changing layout/arithmetic. Native TIME_S stores phase-dependent selection metrics
and sentinels, not physical seconds. UnitScale.time_s remains the physical unit.

The result includes complete raw/clamped projection observations, source row
transition and exact global-cache publication. Explicit sector masks identify
source-defined channels. Active T3 raw barycentrics are defined in all sectors,
but only sector1 clamped/cached barycentrics are defined. Inactive products leave
all barycentric scratch undefined while FAR/PENT/DD/DIST initialization remains
defined. Our unused payload zeros are API values with mask bits CLEAR; they are
never native observations or consumable selected geometry. A future geometry
factory must reject consumption without the required defined masks.

The independent oracle compiles whole original subroutines. Its only numerical
storage observation is a read-only DD copy at return; scratch seeding is separate
debug evidence. NativeArray fields are not inferred from production formulas.
Tests vary finite scratch seeds and verify invalid cache channels stay masked.

Retained geometry includes all Q4/T3 projection, free-edge/vertex/cone branches,
native sector preference and signed sliding-marker encoding. Exact equal-source
node/boundary references require identical scalar representations, including
signed zero; the shared math ScalarBits utility checks values without padding
comparisons or floating arithmetic. Shared main-frame preparation is factored
from the qualified raw geometry code with identical operation order.

Owning CMake includes parent normal/friction/raw-geometry regressions, then native
retained host/CUDA comparisons and header-only consumers. Bazel owns independent
header closure and pinned source preparation. Current tests use the139-point
geometry corpus plus inactive/underflow, T3 scratch masks, old-sector loss,
invalid source/prior state and output preservation. They do not qualify the full
vehicle profile or claim a speed win.

PEN3's focused2064-case/8256-call oracle found target outputs bit-identical between
all-T3 and mixed packets. This does not require reproducing CPU scheduling.
Genuinely stateful row phases, original occurrence identity and per-row native
order remain explicit until the complete pipeline establishes permitted reorderings.

Before vehicle admission, the classifier→cache→raw-geometry path must close the
documented unassigned-XP condition or select an explicit reviewed repair profile.
In particular, side-B barycentric swapping in the forthcoming _22 stage can
reintroduce a tiny negative computed LA after native clamping. No convenience
clamp or invented projection is used here.
