# Native pre-ASSTIFI seed

The host startup accumulator receives two already ordered occurrence spans. Volume/bulk-volume and existing translational stiffness are independent channels. It initializes every node to positive zero, preserves each span's addition order and repeated/zero occurrences, and publishes the output only after every finite sum succeeds. This is failure-atomic publication, not atomic floating-point arithmetic or concurrency support.

No scalar coefficient equation, node sorting, model parser, physical mass/J ledger, owner, clock or GPU work belongs here. Source completeness and native EID/stage/raw-slot order must come from an authenticated caller. Unknown generated IDs or ICONTR corrections cannot authorize a ready app coefficient source.

The explicit shell-source overload requires PhysicalShellsWithNodalSeed and a complete matching seed node span. OrdinaryShellsOnly rejects a supplied seed and retains its exact historical zero-seed path. Seed reads participate in all scratch/output alias checks. The shell loop still provides ETNOD/NSHNOD and calls the existing ASSTIFI leaf once per node. Nodes without incident shells can have solid/beam/spring coefficients.

This nodal extension does not expand the primary-face coefficient profile. Requested main outputs still require separately authenticated ordinary-exterior roles. Coated physical shells may contribute to the whole-model ET/count ledger while their primary coefficient outputs remain unadmitted by this API.

Caller-provided scratch is private and may contain a partial reduction on failure; public outputs and preflight forecasts remain unchanged on failure. Required bytes are forecast before source dereference/allocation; the adapter itself allocates nothing.

Post-ASSTIFI correction is a separate entry point in
`RadiossType25NodalCorrection.h`. It preserves native EightSlot storage order,
first-solid tagging and the subsequent TYPE24/IGSTI-1 global-factor tail. Its
PM32/PM107 and ordered inputs require source binding; coefficients here are
native startup contact K, never structural STI or physical nodal M. Repeated
secondary occurrences remain repeated and virtual secondaries are skipped.
No per-step CPU work is introduced. Both accumulation and correction use this
package's same bounded host arena/range utilities and publish only success.
