# Current normal cache oracle

This adds a small wrapper to the existing type25_startup_oracle target. Its
complete original I25NORMP and I25FREE_BOUND objects, constants, module/COMMON
boundary and Engine MVSIZ129 compile only once. Never link a second copied normal
core. The separately qualified activation oracle currently exports the same
FREE_BOUND symbol, so its regression executables remain separate.

The caller supplies actual current coordinates, coefficients, integer masks,
a complete fresh free-main roster and every prior NOD_NORMAL bit. The wrapper
validates that roster through original FREE_BOUND, copies prior cache, clears
VTX_BISECTOR as MAIN_NORM does, and executes complete NORMP FLAG1 thenFLAG2.
Only source arrays/control setup and observations are added. No production
normal operator supplies expected values. Calls are serial with the shared
startup oracle COMMON environment; this is not a CPU performance reference.

Observations include final face cache and rebuilt LBOUND/bisectors, FLAG1 cache,
actual primary TAGE and native ordered FREE_BOUND records (including zero-normal
sentinel slot3). Unused WNOD is never exposed. T3 slot3 can retain explicit prior
cache bits; that retention is not a claim of fresh native arithmetic on it.
The original source may generate nonfinite results from finite extreme operands;
the oracle reports that fact while production must reject before publication.

Source check runs only text/hash verification. Owning compilation and strict
host/CUDA multi-update tests remain required. Tests must evolve independent
native and production prior caches, never feed a production result into the
native expected recurrence. Required cases include sparse masks, TAGNOD/ACTNOR
independence, rotated/deforming Q4/T3, inactive interior retention, zero raw edge
versus zero transformed direction, boundary counts above2, coefficient-zero
arithmetic, staged failure/caps/alias rejection and separate CUDA barriers.
