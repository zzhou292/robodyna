# Declared native shell-impact source

The bounded scene definition, mesh construction, exact-width native deck and create-only export now have production-neutral modelio ownership. Existing benchmarks/native_contact_scene imports and command remain compatibility entries to the same implementation; no grid or geometry was copied into a second runtime generator. The shared strict JSON reader moved to output/json_object.py; direct accepted_payloads script invocation remains supported.

`python -B -m modelio.native_contact_scene SOURCE.json NEW_DIRECTORY` emits the same source/mesh/native reference files as the benchmark entry. These are declared inputs, not resolved numerical values or accepted physical states. No solver, GPU or renderer is launched.

`PrepareNativeHardening` converts authoritative native plastic H (in SI) to the existing TL tangent input using the exact binary64 association `H/(1+H/E)`. It calls unchanged TL material preparation and records source H, derived ETAN, actual prepared H and ULP difference. It accepts at most2ULP, rejects nonfinite/underflowed/ill-conditioned controls, and never claims exact source-H identity or adjusts a parameter for one scene. The current E210GPa/H1GPa case reconstructs H one ULP high. Full coupled-state/history tolerance remains a separate reviewed gate.

Source/export Python regressions have passed after the move. Material C++ qualification and the physical/contact source factory/archive adapters are separate pending work. This is not yet a complete running case or a vehicle self-contact video.
