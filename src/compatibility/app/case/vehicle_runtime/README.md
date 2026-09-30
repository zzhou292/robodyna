# Complete physical initial startup

`VehiclePhysicalStartup` retains the complete original `VehicleShellExecution`
and `VehiclePhysicalAttachments` from the same actual `VehiclePhysicalModel`
backing. `Preflight` performs no owner allocation. `Prepare` constructs one TL
nodal owner, all six mapped participants and the existing physical publisher.
There is no public mutable owner or advance operation.

The explicit initial velocity is `35.0 * 0.44704` m/s along +X. Coordinates and
raw M/J come from the immutable physical domain and single complete native
ledger. Identity quaternions and zero spin are supplied once. Ordinary nodes
without a rotational source role have absent rotation; source PART/plain and
CIN dependents retain their qualified roles even when M/J or inverses are zero.
Zero is never converted into a fixed reaction constraint.

Packing retires before participant construction. An actual epoch-zero trial
binds Q/T/QBAT/TYPE25/TYPE13 accepted caches and is discarded without a kick,
drift, sealing, or history commit. Solids retain their qualified native TT0
constructor results. The sole existing physical publisher then claims all six
participants. Destruction releases that claim before participants and owner.

`InspectInitial` reads complete native shapes sequentially, checks actual
canonical coordinates, 35 mph velocity, raw M/J, explicit inverse/rotation
roles and CIN saved secondary coefficients, then visits typed 0/1/3/4-point
shell results and connection/solid caches. Rigid skins expose no material
history. Native solid TT0 floating residuals are retained. No source geometry,
material or coefficient formula is duplicated by the app.

The resource report distinguishes the previous source construction bounds
from current retained source and native resident payload. Exact backing checks
precede source sharing discounts. The four shell/spring public forecasts give
exact device bytes and conservative incremental retained/scratch bounds. The
TYPE13/solid complete host bounds are conservatively charged as retained after
subtracting their already retained exact source handles. Serial construction
and readback phases use a maximum temporary bound. CUDA driver/context and
allocator RSS remain outside native explicit-allocation byte counts; the root
runner separately measures RSS and GPU growth. Default global limits are
20,000,000,000 host bytes and 4 GiB explicit device bytes. The configured step is
only a constructor descriptor, not a numerical stability or interval admission.

Owning CMake targets are `robo_dyna_vehicle_runtime_values_check` and
`robo_dyna_vehicle_physical_startup_check`. Run CTest `vehicle_runtime_values`,
then **only** `vehicle_runtime_forecast` and inspect its report before executing
`vehicle_runtime_original_owner`. The original tests reuse the existing source
fixture loader and authenticated physical search/attachment graph. They retain
all 349,645 source shell parents, 5,102 skins, 1,037,877 material points, 4,442
TYPE13 and 2,828 TYPE25 connections, 2,412 solids and 372,435 physical nodes.

The census includes only TYPE13 N1/N2 endpoints, excluding reference N3. Root's
previous independent gate established 8,884 endpoint records/7,493 unique nodes,
all CIN secondaries, with no PART/plain/master intersection. The new original
control also checks all 359,785 shell nodes are disjoint from the 11,165 CIN
secondaries. These role counts do not grant a wall/contact policy.

Author checks are limited to small host packing and C++ syntax. Full source
forecast, owning CUDA compilation, original owner allocation and readback are
root-owned gates. This slice makes no full-vehicle trajectory, joint, beam-tie
activity, contact or crash qualification claim.

The runtime also consumes the explicit extended solid Model through
`PhysicalCinExtendedLaw44Law90V2` (TL resident dependency `66d5d4b`). Its existing
single solid participant retains all five families. Initial inspection requests
the complete LAW36/HEPH/S6Z/LAW44/LAW90 typed buffers together, checks every
family count and every sample-zero stamp, and reports their combined count.
The readback forecast charges that same simultaneous five-buffer payload.
Original three-family models continue selecting `PhysicalCinV1`; both added
buffers are absent for that profile.

This adapter does not select the V4 source/domain or create its joint policy.
Author verification is limited to the three changed C++ translation units with
`NDEBUG`, using the frozen extended resident headers under one CPU/512 MiB.
Meaningful end-to-end validation requires the actual owner: rerun existing
`vehicle_runtime_forecast`, `vehicle_runtime_original_owner`,
`vehicle_runtime_joint_forecast` and `vehicle_runtime_joint_owner` for V1, then
gate the separately composed V4 forecast before its full initial inspection.
The V4 inspection must report 4900 parents with family counts
908/1991/350/306/1345, preserve epoch zero on repeated reads, and match the exact
forecasted device allocation. These V4 runtime checks remain root-owned and
pending this adapter's freeze.

## V5 structural beam participant

The sealed V5 domain requires the exact retained `structural_beams()` Model and
its `ledger.beam18()` contribution. `BeamRuntime` authenticates their shared
backing before forecasting or construction. It uses the native Batch forecast
and discounts only the Model payload already charged by the complete ledger;
the separate Batch handle and all incremental staging remain charged. Beam
readback participates in the maximum sequential temporary reservation.

`InitializeStructuralBeams` owns a separate typed beam18 Batch and its native
TT0 cache. The common publisher claims it together with the existing producers
and optional joints. The existing epoch-zero shell/connection pointer-binding
trial remains unchanged. Reverse destruction releases the publication, beam
and joint batches, then the owner and retained sources. Initial inspection and
`CaptureAccess` authenticate actual beam source/count/stamp and complete typed
readbacks. The app never reconstructs beam force or inertia equations.

The controller's `vehicle_run_supports_forecast` checks exact inclusive host and
device caps plus a 141-parent beam-cap rejection before owner allocation.
`vehicle_run_supports_initial` checks all eight participant kinds, repeated
epoch-zero inspection, actual beam capture identity and failed replacement
without losing the prior owner. The separate two-interval loaded-prefix gate
checks single-owner assembly/publication and authenticated archive/replay.
These full-source/CUDA gates are authored for root execution; author checks
cover only C++ syntax and small CLI/profile/loop values.

Author handoff: 17 host functions pass (12 CLI/loop, one source-policy pairing,
four contact/summary regressions). All 21 changed/new C++ translation units pass
syntax against TL `01a9a39` and archive dependency `6c20dc5`; peak sampled syntax
RSS is 410,763,264 bytes. Reports are `vehicle-supports-runtime-host-tests-1`,
`vehicle-supports-runtime-profile-tests-2`, `vehicle-supports-runtime-report-tests-2`
and `vehicle-supports-runtime-syntax-2`. Disposable direct-link attempts omitted
ArtifactIO/BoundedArrayIO dependencies; their first reports preserve those link
failures, and the passing retries use the actual implementations. Owning CMake
already links both dependencies. No native/CUDA or original-source run was
performed by the author.

Root can reuse the configured `case/vehicle_run` live/original build and pinned
source fixture arguments. Build `robo_dyna_vehicle_run_original_check`, then run
these CTests separately under the workstation guard, inspecting the forecast
before the first allocation:

```
ctest --test-dir BUILD -R '^vehicle_run_supports_forecast$' --output-on-failure
ctest --test-dir BUILD -R '^vehicle_run_supports_initial$' --output-on-failure
ctest --test-dir BUILD -R '^vehicle_run_supports_loaded_prefix$' --output-on-failure
```

Also rerun `vehicle_run_values`, `vehicle_run_reports` and the existing V1/V4
forecast/loaded-prefix gates as affected regressions. The loaded test can retain
its archive via the existing `ROBO_VEHICLE_RUN_OUTPUT` empty-directory option.
