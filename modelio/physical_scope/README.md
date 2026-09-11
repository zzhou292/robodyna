# Original vehicle physical-source coverage

`PhysicalScope::Prepare` takes the existing immutable rigid point-mass, tied
declaration, TYPE13 and selected-solid source handles. The rigid source retains
the complete tire-free `VehicleSourcePlan`. It preserves one canonical backing
and returns source membership and incidence; it constructs no owner or mass row.

The baseline union contains the 349,645 retained shells, the N1/N2 endpoints of
all 4,442 TYPE13 beams, all 2,412 selected adhesive/rubber cells, and the 54
point-mass nodes selected for original rigid PARTs. N3 has an explicit orientation
role and enters the baseline only through a separate physical producer.

The second union adds every original `CONSTRAINED_SPOTWELD_ID` endpoint as a
**provisional source-only** role. Shared fixed-width source-card helpers read IDs;
the existing component's blank-default subset is identified without interpreting
optional strength/failure fields. Complete raw cards and source lines remain in
the tied declaration. No TYPE25 property, coefficient or runtime admission is
created, including for a default-only row.

Every one of the 759 plain groups and 20 PART roots retains its complete ordered
member list with before/after coverage and exact excluded-tire incidence. The
22 primitive PARTs and two merges remain in the original rigid handle. All 5,102
rigid skin parents retain their original EID, PID, canonical row and PART root;
this is independent of future material execution-role authorization.

The report includes full original shell/beam/solid incidence for group members,
weld endpoints and all 155 point-mass nodes. The 101 mass records outside the
current rigid selection remain visible even when their nodes are already covered
by another producer. A covered node does not mean that every incident producer's
mass or attachment is retained. In particular, complete group coverage does not
authorize silently omitting source point masses or connected excluded tire parts.

Before allocating, the factory charges the existing inclusive rigid/point-mass
source reservation, additional owned tied/TYPE13/solid payload, bounded serial
array-decoder workspace and the complete result. The old source reservation
conservatively includes retired parsing; it is not reported as retained payload.
Canonical backing is charged once. The result's owned payload includes a control
reserve and vector capacities, excluding allocator/RSS. The default cap is
512 MiB. Report serialization is a separate caller operation with a 32 MiB output
cap; its transient string buffer is not part of `Prepare`'s forecast.

The four tiny tests cover literal/optional cards, late malformed records, caps,
missing nodes, provisional roles, tire/orientation exclusions and unchanged
published values after failure. The optional original target checks full source
counts, all-group incidence, exact-budget retry and shared backing. It can write
the census to a new path supplied by `ROBO_PHYSICAL_SCOPE_REPORT`.

Owning original gate:

```sh
cmake -S <app>/modelio/physical_scope -B <build> \
  -DCMAKE_BUILD_TYPE=Release -DChrono_DIR=<Chrono>/lib/cmake/Chrono \
  -DROBO_DYNA_TL_ROOT=<TL> -DROBO_DYNA_PHYSICAL_SCOPE_ACTUAL_TESTS=ON \
  -DROBO_DYNA_PHYSICAL_CANONICAL=<crash-work>/assets/yaris-vehicle \
  -DROBO_DYNA_PHYSICAL_SCOPE=<crash-work>/reports/yaris-full-shell-scope-10.json \
  -DROBO_DYNA_PHYSICAL_DECLARATIONS=<original-vehicle-declarations.json> \
  -DROBO_DYNA_PHYSICAL_TYPE13_DECLARATION=<crash-work>/reports/yaris-type13-startup-declaration-1.json \
  -DROBO_DYNA_PHYSICAL_REPORT=<new-report.json>
cmake --build <build> --parallel 1 --target \
  robo_dyna_physical_scope_fields_check robo_dyna_physical_scope_source_check
ctest --test-dir <build> --output-on-failure
```

Root schedules the original execution. No original-count result is claimed by
the author-only tiny gate.
