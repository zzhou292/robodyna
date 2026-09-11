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
