# Selected native TYPE25 endpoint assembly

`RadiossType25Assembly.h` exposes small numerical leaves for the observed
nonthermal, non-pinch, local `I25ASS0` profile. It consumes the qualified normal
and friction response. It has no solver clock, physical owner, device allocation,
contact-area law, mass scaling, remote-node buffer, or independent commit method.
Production is C++/CUDA; the native Fortran donor is a qualification-only target.

Source is OpenRadioss `a62b27e6baa555d222a580d6218867d0be4d70b5`, Siemens 2026:
`i25for3.F` endpoint tail (3200–3222) and `i25ass3.F` selected `I25ASS0`
main loop (486–509) and secondary loop (545–567). License and attribution remain
in the parent [LICENSE.md](../LICENSE.md). The exact source files and mechanically
extracted oracle blocks are pinned by the owning qualification manifest.

## Responsibility and order

- `Endpoints.h` prepares native main-slot products and final stiffness from
  **post-response** interpolation weights. The left-associated `HH` sum gates
  assembly; zero `HH` does not read undefined force/stiffness scratch. The
  secondary resultant is kept with its original sign and subtracted at gathering.
  The SI bridge converts completed native products once. Native history stays native.
- `Schedule.h` describes the original logical source cohorts. Each cohort orders
  all rows' four main slots first, then all secondary rows. Cohort sizes are not
  CUDA launch sizes. Native worker/global reduction order remains a separate
  whole-engine comparison; this component makes no cross-worker bit-identity claim.
- `Incidence.h` reuses `BuildOrderedNodeIncidence` for a complete immutable CSR.
  Repeated nodes and zero-weight slots remain distinct ordered occurrences.
  The caller supplies correctly sized host buffers; malformed maps, schedules,
  capacities and input/output overlap fail before either output changes.
- `Gather.h` folds one node from the **actual incoming** force and stiffness.
  One device thread owns each node; no floating-point atomics or parallel
  within-node reassociation. Native zero additions, signs and repetitions remain.
  A late invalid/nonfinite sum leaves that node's staged output untouched.

The caller must authenticate the source maps, complete CSR and packet generations,
provide readable host/device spans, and keep them immutable through the kernel.
Rebuild/authenticate incidence whenever contact associations or cohort ordering
change. This leaf checks each visited occurrence but cannot prove completeness
of a forged CSR or validate physical allocation bounds from raw pointers.

All nodal results remain in separate trial scratch until **every** node succeeds
and the common physical owner accepts the attempt. A successful node is not a
publication permit. Rigid/CIN transfer, rotational channels, timestep screening,
common discard/retry and device resource forecasts belong to that integration.
The empty contact roster is explicit and preserves incoming values unchanged.

## Qualification boundary

`lib_utest/qualification/radioss_type25_assembly` compares complete initialized
native nodal arrays against independently compiled unchanged Fortran loops.
Inputs include inactive scratch, cancelling/negative/zero weights, repeated and
aliased endpoints, nonzero incoming arrays and distinct logical cohort sizes.
CUDA runs the same packet leaves and one-writer gather under varied launch sizes.
Failure/retry tests exercise private per-node staging, not a physical owner commit.

This is not a complete GPU contact coordinator, source-binding implementation,
remote/SPMD assembly, thermal/pinch profile, whole-engine speed qualification or
vehicle acceptance. The actual engine probe did not read NSPMD; local source maps
must be authenticated before this endpoint profile can be selected.
