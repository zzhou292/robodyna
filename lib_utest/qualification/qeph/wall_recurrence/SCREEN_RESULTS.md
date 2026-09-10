# CW1 actual full-state wall screen: passed

The authenticated six-job reader and all four signed boost comparisons passed
on first execution. Every sampled step passes all six jobs and four comparisons.
The unchanged factor-two margin selector chooses **H0 = 2^-24 s**
(59.604644775390625 ns). Diagnostic 4H0 also passes; it is not needed for selection.
This admits the named next fixture gate, not a nonlinear impact trajectory or a
vehicle timestep. Full incoming/rebound/refinement remains separate.

The final index is `crash-work/runs/qeph-wall-selection-1/index.json`, SHA256
`b803dcdbd7f549965839b940af0303c46046ab16f12f54b43fc7fa9efb622832`. It pins all four full boost comparison files, selection.json,
provenance and progress. All 18 files (653,018 B) verify. The complete raw,
derived and selection set totals 90,742,327 B, leaving 9,920,969 B under 96 MiB.
The 57,988-byte launch configuration SHA is
`d3f2561cc9b529e158d017dc7a8a10ed7e385674fed3f2dc740e6e5fbe4b6bb3`;
it binds 234 reviewed inputs and 66 declared linked inputs. Actual executable:
`662de66de806be9f0fe3b75619b261ef315b4a899a23d93a68960c2b49a4eb74`.

Maximum absolute moving-baseline error is 0.
Maximum directed matrix-difference/budget ratio is 2.5996912955116178e-06;
maximum weighted-gain-difference/budget ratio is 2.2481915674221694e-08.
All complete 109/194-coordinate observations and previously measured spectra/
Gram systems are retained. No native recurrence or eigensolve was repeated.
The earlier derived runs give maximum weighted mean gain 45.546286387450365
below 64 and constant-branch radius 1.0000000047784086 below 1+5e-8.

The serialized command took 1.56 s with sampled peak process-group
RSS 41,123,840 B, within one CPU/1 GiB/120 s. No GPU was used.
All guards passed. The utility source/runtime is qualified at TL `af05bc4`;
`qeph-wall-screen-1` preserves it with this actual decision. Raw and derived
operand checkpoints remain separate dependencies. Compiler/native toolchain
scope remains explicitly delegated and nonhermetic.

CW2 must verify this final index and its linked passing decision before launch,
then pass `TL_CW2_SELECTED_H=0x1p-24` and `TL_CW2_SCREEN_SHA256` equal to the
index SHA above. The CUDA fixture checks that supplied binding; the root launcher
authenticates the actual archive. No fallback or silently selected step is allowed.
