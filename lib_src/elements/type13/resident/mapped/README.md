# TYPE13 on the complete physical owner

`ForecastMapped` / `InitializeMapped(config, physical, rigid, owner, cin, limits)`
consume the exact TYPE13 model from the immutable physical ledger. There is no
second independently supplied model or mass source. `AssembleMappedAccepted`
uses the actual owner token, raw current coefficient authority and CIN stiffness
view. The ordinary `InitializeJoined` / `AssembleAccepted` / private attach path
remain strict; changing the shared legacy rigid-scope predicate is unnecessary.

Before optional resident allocation, the initializer checks complete extents,
local and inclusive budgets, the actual rigid binding, endpoint roles, full
initial kinematics/raw M/J and the complete CIN roster through
`ShellPhysicalOwner`. Mapped state retains immutable physical/rigid handles,
actual owner identity, witness count and the authenticated initial source view.
Common publication accepts only this mapped scope and performs all fallible
preflights before claiming. Accepted reads protect the complete retained
physical and rigid payloads, including non-beam source tails.

The original-source current qualifier excludes rigid-member or fixed / absent
rotational TYPE13 endpoints through the actual owner queries. This excludes zero
of the original 7,493 endpoint nodes: app3837760 receipt
`cin-master-role-root-functions-2/robo_dyna_tied_cin_actual_check.xml` reports zero
PART/plain/auxiliary intersections across all 11,165 CIN secondary nodes,
including those endpoints. This is a source-specific receipt, not a general
claim that beam endpoints never belong to rigid groups. General rigid endpoints
remain a separate qualifier. Unrelated solid-only absent rotations are admitted.
CIN-dependent inverse mass/inertia may be authentically zero; no positive inverse
or reconstructed raw coefficient is required. Per-attempt device checks preserve
this same endpoint role contract.

Both initializers share one TT0 packing function and one upload function.
Both assembly paths call the same ordered force/couple and pre-floor stiffness
kernel; only the mapped endpoint role check and token authentication differ.
Candidate recurrence, current-versus-saved force behavior, native OFF stiffness,
work arithmetic, arena layout and the sole accepted/trial selector are unchanged.
The common publisher still commits the owner once, then performs infallible
participant slab swaps. No independent beam clock or publication is introduced.

## Budgets

Existing `BatchLimits` count/device/local-source host caps remain enforced.
`BatchMappedLimits::max_host_bytes` separately caps the simultaneous complete
physical reservation (256 MiB default; explicit `Vehicle()` is 2 GiB). The native
arena remains bounded at 32 MiB and 8,192 connections / 524,288 domain nodes.
The mapped reservation includes private state, retained physical/rigid payload,
TT0 host arena, readback staging and the existing initial-proof scratch. The
source model backing is already owned by the exact physical ledger. When the
physical execution binding owns the exact same nonempty rigid group/member
arrays, that backing is charged once; equal values alone do not earn a discount.
Without that shared execution authority, the two inclusive payload estimates are
conservatively charged, including any nested ledger overlap. Payload accounting
excludes allocator overhead/RSS, like the existing arena contracts.

The tiny complete all-family fixture forecasts 492,536 B host and 3,204 B device.
This is not an original-count owner measurement. The original full composition
must call the same checked forecast before allocation under the root guard.

## Qualification

Author gate `type13-mapped-author-2`: six physical/mapped host functions and five
unchanged TYPE13 host functions pass, together with source identity and all
host-callable production/fixture syntax checks. Guard: one CPU, 512 MiB ceiling,
14.534 s, 400,531,456 B sampled peak RSS. No native runtime or CUDA execution was
performed in the author lane.

The complete fixture retains its PART, plain group, CIN dependent, unrelated
rotation-free solid nodes, coincident Q layers and all six actual participant
kinds. Two additional CUDA functions test preallocation legacy/budget/owner/
roster failures and exact retry, zero dependent inverse coefficients, output
alias rejection and a last-endpoint role failure followed by a common commit.
The original three common CUDA functions still compare every participant and
owner history through failed attachment, rejected capture, late solid failure,
discard and retry. The existing native/actual 4,442-beam trajectory gate remains
the independent unchanged-recurrence regression.

Root commands (fresh build directories, serialized guard):

```sh
cmake -S <TL>/lib_utest/qualification/physical_publication -B <build> \
  -DCMAKE_BUILD_TYPE=Release -DPHYSICAL_PUBLICATION_CUDA=ON \
  -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build <build> --parallel 1
ctest --test-dir <build> --output-on-failure
```

This owns 6 physical/mapped host + 5 existing TYPE13 host + 5 actual CUDA
functions. Existing independent native and original trajectory qualification:

```sh
cmake -S <TL>/lib_utest/qualification/type13_resident -B <legacy-build> \
  -DCMAKE_BUILD_TYPE=Release -DTYPE13_RESIDENT_NATIVE=ON \
  -DTYPE13_RESIDENT_ORIGINAL=ON -DTYPE13_RESIDENT_CUDA=ON \
  -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build <legacy-build> --parallel 1
ctest --test-dir <legacy-build> --output-on-failure
```
