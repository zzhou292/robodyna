# Original CIN source-to-domain map

`TiedCinAttachments::Prepare(post_kinchk, domain, limits)` retains the immutable
original source receipt and builds the TL reference attachment map. Source rows
come from the complete finalized NSV list and one-based selected IRECT rank.
Declared master EID/PID comes from `TiedShellDeclaration`; optional coincident
INCOQ material witnesses cannot rename the mechanical patch.

Canonical represented SI positions are decoded through the existing bounded
codec. Each consumed original working coordinate is multiplied once by the
declared length scale and checked bitwise against canonical SI. No SI-to-working
round trip or duplicate geometry source is introduced. The supplied domain may
contain additional nodes, but this map proves only the required tied-node
closure. It does not authorize other participants, full-vehicle DOFs, physical
coefficients, current motion, failed-master behavior or an owner.

Preflight retains the source/classification receipt and actual post-KINCHK
backing, then adds the TL model/domain/index reservation, complete input rows and
the bounded canonical decode peak (56 bytes per original canonical node).
Retired classifier/native scratch is not added again. The same post handle is
charged once. Each new shared allocation reserves 64 bytes for shared control;
the staged TL model and retained app storage handles are charged. The default
app cap is 512 MiB; the original-count forecast is printed before construction.
Payload forecasts exclude additional allocator metadata and are not RSS predictions.

Three host functions cover source identities, stale working bits, finalized tail
errors/retry and exact budget boundaries. The older tiny source fixture has
collinear masters; its explicit reference rejection is retained. The original
gate supplies a reversed node domain over the geometric source union, checks all
11,165 mappings against canonical source IDs and existing native patch geometry,
then removes the final required secondary node and verifies the exact failure
row/source ID before retry. That domain is a geometric qualification input, not
a complete vehicle DOF inventory.

```sh
cmake -S case/vehicle_startup/tied_cin -B <build> \
  -DROBO_DYNA_TL_ROOT=<TL-checkout> -DChrono_DIR=<Chrono-config> \
  -DROBO_DYNA_TIED_CIN_ACTUAL=ON -DCMAKE_BUILD_TYPE=Release \
  -DROBO_DYNA_TIED_CANONICAL=<canonical-directory> \
  -DROBO_DYNA_TIED_SCOPE=<yaris-full-shell-scope-10.json> \
  -DROBO_DYNA_VEHICLE_DECLARATIONS=<yaris-vehicle-declarations-1.json>
cmake --build <build> --target robo_dyna_tied_cin_values_check robo_dyna_tied_cin_actual_check -j1
ctest --test-dir <build> -R '^tied_cin_' --output-on-failure
```

Author evidence: three host functions and public/original-test C++ syntax pass.
The two original-source functions require the owning CUDA/native lane because
they reuse the already-qualified complete upstream search/classification fixture.
