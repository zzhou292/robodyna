# CW1 analysis utility results

All 13 new analytical functions and 18 retained model/support functions passed
on first execution. The new functions qualify full-state metric, chronology,
Schur/Gram, identity and comparison operations against independent small
systems; they do not execute the full native contact screen.

The tests retain Jordan drift and passive identities, verify the oscillator's
modified quadratic form, reject wrong spring signs and unit coefficients,
and expose an unstable alternating sequence whose individual branches have
stable spectra. Literal state sums independently check chronological order,
empty runs and the single final endpoint. Six scalar step schedules agree
with an independent two-position recurrence and preserve all nine event
windows. Complete observer/cache signs, failed baseline evidence and the
fixed coarse-step selector are checked without reducing the native state.

Raw and weighted numerical evidence remains distinct. Only weighted mean gain
and weighted gain consistency control those decisions; raw drift remains
diagnostic. Both Schur calculations and both Gram calculations must resolve.
The exact policy and test budgets are in
[ANALYSIS_CONTRACT.md](ANALYSIS_CONTRACT.md).

The guarded one-worker CMake build took 14.522 s and reached a largest sampled
process-group RSS of 759,308,288 bytes. All resource guards passed. Tests used
one CPU and a 1 GiB RSS cap, with no GPU. The new analytical test bodies took
0.005 s; sampling of such brief tests is not an exact peak-memory measurement.
Reports are `crash-work/reports/qeph-wall-analysis-*-1.json` and
`qeph-wall-analysis-xml-1/` in the parent workspace.

The [pre-execution input map](analysis-source-map.json) records 119 inputs,
SHA-256 `d0732a70a7c62f37c03a4dc1da530f8d0d132cfb69b1be1eaf7bdf20f6dd62b5`.
The `qeph-wall-analysis-1` checkpoint retains sources, actual test binaries
and reports. Native/compiler dependencies retain their declared delegated
scope; this is not a hermetic toolchain image.

Full native capture and amplitude/boost comparisons, actual Schur/Gram
decisions for the named contact model, authenticated reports, moving-startup
qualification and nonlinear CUDA impact/refinement remain pending. No impact
timestep, mixed-shell dynamics or vehicle response is admitted here.
