# Original TYPE45 source

`VehicleType45Source::Prepare` retains the immutable physical source/domain,
all 44 original regular joint rows and their exact original source cards.
The explicit `OriginalDirectSdiType45V1` policy resolves three shared
properties with ScF=.01, Cr=.05 and zero free stiffness/viscosity. It preserves
blank RPS/DAMP and the first original declaration supplying each kind.
It does not invent generated native property or body-main node IDs.

The source rule follows the pinned direct in-memory Dyna-to-Radioss path,
not optional text-export DEFAULTS. `readCell_VALUE` and
`readCell_SCALAR_OR_OBJECT` retain blank numeric fields as absent;
`convertutils.h` starts an absent double at zero, so the regular converter
sets ScF=.01. It never writes Cr or free K/C. `ReadDynaAndConvert` passes that
same model directly to `GlobalModelSDISetModel`. The dimensional SDI getter
returns absent fields unavailable/zero; HM_READ_PROP45 changes Cr=0 to .05.
The CFG's RPS=1 and free K/C=1 defaults therefore do not fill this profile.
Full source closure and exact pinned file/line references are recorded in
`planning/YARIS_TYPE45_SOURCE.md`; the source cache manifests 7–12 use pin
`a62b27e6baa555d222a580d6218867d0be4d70b5` with SHA256/Git-blob checks.

N1/N2 are force endpoints; N3 is the initial axis for Type2/3. N4 is retained
original evidence, unused by this native three-node conversion. The
cylindrical CFG marks columns 5/6 as `_BLANK_`, but four original cards contain
literal integers there. `unused_columns` preserves them without claiming
extra node roles or connections. Complete raw text remains in the retained
rigid source object.

Required 38 versus boundary 6 is explicit. Boundary IDs are 2200514/515,
2200526/527 and 2200528/529, each with one endpoint in an omitted assembly.
Original plain/PART memberships are source identities, not native registration
or proof that a node is not a rigid main. The 76 required endpoint occurrences
have 48 plain-source and 28 PART-source memberships. `Geometry()` exposes exact
SI source coordinates. No native dynamic Reference is created: the actual
owner must supply TT0 main-node M/J/K/Krot, body mass/mean-principal J and its
selected/clamped target dt. No force, history, timestep or common-transaction
claim is made by this source object.

All retained backing, coordinate decoder copies, mapping workspace and new
rows are forecast before allocation under the existing 512MiB source cap.
Mapping publishes only after complete source/domain position-bit checks,
including signed zeros; copied source objects retain their backing.

Four tiny host functions pass; the complete original 2-function source test is
authored for root execution. Its expected census is 44/38/6, 372435 domain nodes,
three shared properties and eight literal unused cylindrical values. It also
checks exact-cap rejection and immutable copy lifetime.

Owning configure (use the qualified TL tree containing TYPE45):

```
cmake -S modelio/type45 -B BUILD -G 'Unix Makefiles' \
  -DCMAKE_BUILD_TYPE=Release \
  -DChrono_DIR=/home/jsonzhou/Desktop/chrono-work/crash-work/install/chrono-vsg-r0/lib/cmake/Chrono \
  -DROBO_DYNA_TL_ROOT=/home/jsonzhou/Desktop/chrono-work/Total-Lagrangian-FEA \
  -DROBO_DYNA_VEHICLE_TYPE45_ACTUAL_TESTS=ON \
  -DROBO_DYNA_PHYSICAL_CANONICAL=/home/jsonzhou/Desktop/chrono-work/crash-work/assets/yaris-vehicle \
  -DROBO_DYNA_PHYSICAL_SCOPE=/home/jsonzhou/Desktop/chrono-work/crash-work/reports/yaris-full-shell-scope-10.json \
  -DROBO_DYNA_PHYSICAL_DECLARATIONS=/home/jsonzhou/Desktop/chrono-work/crash-work/reports/yaris-vehicle-declarations-1.json \
  -DROBO_DYNA_PHYSICAL_TYPE13_DECLARATION=/home/jsonzhou/Desktop/chrono-work/crash-work/reports/yaris-type13-startup-declaration-1.json
cmake --build BUILD -j2 --target robo_dyna_vehicle_type45_fields_check robo_dyna_vehicle_type45_source_check
ctest --test-dir BUILD --output-on-failure -R vehicle_type45
```

Root owns these complete-source/heavy gates. Author evidence:
`vehicle-type45-fields-build-3`, `vehicle-type45-fields-tests-2`,
`vehicle-type45-source-syntax-2`, `vehicle-type45-configure-2` (both owning
targets enabled), and the small
streamed pinned-card extent audit `vehicle-type45-card-extents-1` under
`crash-work/reports`.


The explicit native V6 support-source policy preserves the V5 physical node,
rigid group, point-mass, structural-beam and joint populations while binding
`NativeConvertedSupportsV6` solid references (raw8 HEPH aliases). Shared
`HasVehicleSupports` predicates centralize complete-support admission. V5 stays
explicit and unchanged; mismatched V5/V6 source/domain/joint policy combinations
are rejected. This layer does not enable mechanics, a runtime owner or CLI mode.
