# Explicit vehicle-support source composition

`physical_domain::Policy::RetainedShellAssembliesVehicleSupportsV5` requires
`PhysicalScope::PrepareVehicleSupports` with the same authenticated canonical
backing for `solid_source::Policy::OriginalVehicleSupportsV5` (4,980 cells in
five families) and `beam18::Policy::OriginalCircularFourPointLaw44V1` (142 beams).
The old scope entry point rejects this incomplete solids-only combination.
V1 and V4 scope/domain entry points retain their existing selection rules.

The source census adds `Beam18Endpoint=128` for actual N1/N2 only. All 147 beam
source nodes, including N3 orientation evidence, receive complete original EID,
PID, row and slot incidence. N3 alone creates no physical role, mass or inertia.
V5 requires source-complete groups affected by beam endpoints and the six
already required V4 groups. The four rod point cards and the two antiroll cards
are checked by their original EID/NID pairs. The resulting 154 retained mass
cards and all 44 required TYPE45 joints remain explicit source controls. Final
domain and group counts are measured by the owning original-source gate.

`VehiclePhysicalModel::structural_beams()` retains a distinct immutable TL beam
Model with owned material curves. Existing `beams()` remains the TYPE13 model.
A `Beam18NodeContributions` snapshot feeds the explicit V5 coefficient ledger
after the other typed producers. PART and plain groups consume this same
ledger. Plain V5 uses `InitializeNativeTotal`: the beam's native total scalar J
is retained in `unpartitioned_native_inertia_kg_m2`, never reconstructed by
subtracting physical/added inertia. Legacy profiles still use
`InitializePhysical`.

`Limits::VehicleSupports()` supplies the existing 64 MiB extended-solid ceiling.
The additional 32 MiB beam Model and 32 MiB contribution reservations include
their native retained-domain backing and apply only to V5. Source composition
charges the existing canonical backing once, then the beam source's own
payload; later native-phase budgets deliberately overcharge shared handles.
Complete preflight precedes construction. Native per-module caps are still
enforced and a failed replacement leaves the prior immutable handle valid.

The source role resolver authenticates structural beam Model/snapshot/domain
identity. Owner packing requires the runtime beam-role bit to agree exactly
with actual ledger occurrences and retains the existing dependent and absent
rotation rules. The actual V5 CIN gate rebuilds the original search/witness
proof and checks the existing no-CIN/rigid intersection contract, before host
packing. It creates no owner, force cache, time step or trajectory.

## Qualification commands

Use the existing `physical_model` CMake entry point and original fixture
arguments. Add `-DROBO_DYNA_VEHICLE_SUPPORTS_MODEL_TESTS=ON` and build
`robo_dyna_vehicle_supports_model_check`; run
`ctest -R '^vehicle_supports_physical_model_original$' --output-on-failure`.
Eight functions cover source roles/evidence, all canonical coordinates and
selected memberships, typed Model curves/references, native endpoint
coefficients, all joint bindings, complete budgets and late-failure retry.

For the separate original CIN/host-packing gate add
`-DROBO_DYNA_VEHICLE_SUPPORTS_CIN_TESTS=ON`, use the installed CUDA compiler and
build `robo_dyna_vehicle_supports_cin_check`; run
`ctest -R '^vehicle_supports_physical_cin_original$' --output-on-failure`.
It uses the existing classification fixture to authenticate auxiliary/wall
source inputs. Root owns this full-source/CUDA-dependency gate.

Affected small owning tests remain `vehicle_physical_domain_fields` and
`vehicle_type45_fields`, with new support-policy controls. Existing V1/V4
original-model and runtime-packing tests remain required regressions. App run
factories, beam resident/common publication, wall metadata and case admission
are separate integration work; this slice does not admit the full V5 runtime.

Author boundary: seven profile host functions pass, including two new V5
controls and five existing V1/V4 controls. Thirty changed/new C++ translation
units pass syntax checks, and the owning host CMake configuration passes.
Reports are in `crash-work/reports/vehicle-supports-author-1` (host-tests-2,
syntax-1, solid-syntax-1, configure-1); maximum sampled author RSS was
388,304,896 bytes under the 512 MiB guard. The first disposable host link omitted
ArtifactIO; host-tests-1 preserves that failure and host-tests-2 links its actual
implementation. No original-source, native or CUDA execution is claimed here.
