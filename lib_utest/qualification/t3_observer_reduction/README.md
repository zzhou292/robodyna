# Mapped T3 observer reduction

This changes only successful mapped T3 diagnostic reduction. The material
kernel and complete `T3BatchDiagnostics::Measure` body remain unchanged. Root's
pre-change loaded eight-interval run measured T3 evaluation 1.065 s/attempt; that
includes material evaluation, diagnostics and synchronization. No speedup is
claimed until root measures the complete stage.

The six reassociated sums are internal_work[0/1], their two increments,
internal_kick_work and internal_drift_work. Six extrema and all source/phase/
error identities remain exact. Joined kinetic observations remain unavailable.
No sum supplies material history, forces, stiffness or timestep. Existing
accepted/prepared diagnostic comparisons remain exact within one live attempt.

Neutral helpers under `elements/mapped_shell` retain the QEPH 128 B summary,
eight channels and fixed 128-thread block tree. T3 leaves channels 4/5 unused,
without adding hourglass fields to its diagnostics. QEPH keeps the same field
layout, leaf observations and frozen serial oracle. The test-only binary128
sum/bound utility is shared independently of production arithmetic.

After unchanged parallel material evaluation, one preparation kernel validates
parent results and each node's exact nested-hypot displacement/quaternion.
Failed material slots are never inspected. Preparation reuses the typed gather
parent/node scratch after accepted assembly has completed on the same stream.
Up to 256 blocks fold fixed source-index subsequences and use a fixed halving
tree. One final 128-thread block folds those summaries. No floating atomics,
per-parent partial arrays, per-attempt allocation or additional synchronization
are introduced. Launch errors are checked before subsequent stages.

T3 observes all three already-rounded signed cache-work terms through the
existing `AccumulateInternalWork<3, WorkObservation>` expression; the default
double leaf and all mechanical callers are unchanged. It does not replace the
three terms with a pre-summed parent value. Saved OFF=0 still contributes actual
accepted-cache work. Only the explicit RigidSkin law skips parent observations.

Any element/result/node failure, absent catalog, nonmapped or kinetic-scope mismatch,
nonpositive native dt, nonfinite term/seed/intermediate sum or inadequate
finite-prefix proof invokes the complete original finalizer and Measure from
the original identity. Element failures precede catalog/result/node/observation
failures exactly as before. First indices and all named partial failed-control
fields remain exact. Zero/negative dt is not newly rejected. Unused whole-domain
velocities are not newly checked; consumed nonfinite work still fails.

For N=4*parents+1 and maximum absolute rounded term or seed M, the fast path
requires M<=DBL_MAX/(8*N). This existing sufficient QEPH bound conservatively
covers three T3 terms per parent. It keeps both serial prefixes and tree partials
below DBL_MAX/8 including binary64 roundoff. Failure of the proof requests the
serial path, which preserves both admitted huge cancellations and original
prefix-overflow rejection.

The independent binary128 oracle reconstructs rounded three-slot leaf terms.
For B blocks, R=ceil(parents/(128*B)), its addition-depth bound is
`d=3*R+7+ceil(B/128)+7+1`. With u=2^-53, absolute term sum A and
`K=3*parents+2*B*128+2*128+1`, each sum is checked against
`2*(d*u*A+K*denorm_min)/(1-d*u)`. This covers cancellation and gradual underflow,
plus the much smaller binary128 oracle error. Each packet also rejects a
representable corruption outside its own bound. No generic relative tolerance,
mechanical comparison or public interval-certificate rule is relaxed.

Storage adds at most 32,768 B summaries, one 8 B pointer and one 24 B region descriptor.
Actual fixed-header/layout sizes and alignment are counted by existing bounded
forecasts, including complete host startup staging. The old gather sizes in its
host test are updated only for this explicit additional region. Legacy profiles
allocate no observer arrays; their fixed-header growth remains charged. Existing
caps and resource guards do not change.

Seven host and four CUDA functions are authored. They cover partial/full trees
through 33,001 parents; masks and accepted endpoints; one-point/layered rows,
coincident layers and all skins; signed cancellation/subnormals; exact errors
and partial failed records; prefix overflow, negative dt, unused velocities,
removed accepted-cache work, repeatability, immutability and retry. The frozen
oracle includes the complete pre-change T3 finalizer and Measure. Source checks
compare the unchanged material/Measure bodies and authenticate the frozen body
before any numerical test. Existing QEPH observer, gather and Q/T/native/common
publication gates remain required regressions.

Author validation is source identity and C++ syntax only (including temporary
launch-stripped CUDA bodies). Root owns host numerical, NVCC/native/GPU and
complete loaded-source promotion:

```
cmake -S lib_utest/qualification/t3_observer_reduction -B BUILD \
  -DCMAKE_BUILD_TYPE=Release -DT3_OBSERVER_CUDA=ON
cmake --build BUILD --parallel 2
ctest --test-dir BUILD --output-on-failure
bazel test //lib_utest/qualification/t3_observer_reduction:host
bazel build //lib_src/elements/t3:batch //lib_src/elements/qeph:batch
```

Rerun QEPH observer/gather, T3 gather, mapped Q/T native/CUDA and the complete
physical publisher/loaded accepted-prefix retry gates. Histories, force caches,
nodal trajectories, activity, exact extrema and selected step remain unchanged;
only the six observational sums use their explicit bounds. Use existing stage
timers for actual throughput; no new performance timer or solver is added.
