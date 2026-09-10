# Full-shell visualization records

This is the first host-only format foundation. It does **not** publish a full
simulation archive, run a solver, authenticate a live accepted owner, or add a
new AcceptedReplay schema dispatch. Synthetic record tests are formatting and
identity evidence, not crash-physics evidence.

The shared `output/BoundedArrayIO` layer reuses ArtifactIO hashing and checked
create-only writes. It retains the existing canonical raw little-endian
`<u2/<u4/<u8/<i4/<f8>` representation and strict shape/bytes/SHA/field metadata.
Byte counts and scalar types are checked before borrowed reads; finite
binary64 values retain their bits, including signed zero and subnormals.
Relative canonical paths are supported, with real parent directories and
regular files required. Calls and filesystem access are externally serialized;
this layer does not promise concurrent-writer exclusive creation.

`Context` owns immutable original EID/PID, source ELFORM, opaque resolved native
family and point-applicability declarations. Its move constructor retains both
handles, and assignments are disabled. It binds ordered declarations with a
padding-free little-endian SHA. The caller must supply and authenticate the
original source/mapping identity; constructing a context does not establish
that a material or formulation is mechanically supported. Nonplastic and
unavailable fields have no stored PLA values. Native point counts are retained
separately; neither state is converted to zero plastic strain.
Resolved rigid/nonplastic parents may have zero constitutive integration points;
only an available native PLA field requires a positive native point count.

`WriteFrame` writes exact endpoint positions and applicable native equivalent
plastic strain into separate bounded arrays, followed by a small versioned
JSON descriptor. It preflights all values and destinations before the first
write. I/O failure can leave incomplete evidence; only the future archive
writer may publish a run completion manifest. `ReadFrame` requires an expected
complete phase from the authenticated frame index in addition to source context
and file hash. It returns a fully staged value, so assigning its result leaves
an existing frame intact on error. This first profile supports the existing
fixed-dt staggered-half-kick phase, including the initial half kick.

The records are **visualization plus selected fields, not restart data**. They
omit velocities, orientations, full constitutive/stabilization history, and
constraint/contact recurrence. No kinetic or global energy interpretation is
invented. The future live adapter must get values from the sole accepted TL
owner and common shell/connector publication after commit.

`PlanArchive` accepts a configurable saved-frame count and the case's actual
interval count/fixed timestep/requested horizon. It checks their association,
reserves one extra prefix frame, and charges all field arrays, interval records,
optional channels and a separately declared static/metadata reserve. Declared
static files must fit their individual caps and the reserve, with final
manifest/index/configuration reservations present. The reserve is not proof
that a future publisher has listed every source file.

For 359,785 nodes and a conservative 3-point pool for 349,645 parents:

| Profile, 20 ms target | Forecast bytes, including 192 MiB reserve |
| --- | ---: |
| 100 saved + 1 prefix, `h=2^-24`, 335,545 intervals | 2,025,674,952 |
| 100 saved + 1 prefix, `h=2^-26`, 1,342,178 intervals | Rejected above 2 GiB |
| Explicit 88 saved + 1 prefix, same `h=2^-26` and all points | 2,135,428,608 |

The lower cadence is an explicit caller choice. No automatic field dropping,
timestep increase, trajectory interpolation or compression assumption occurs.
The 39-value/312-byte interval core is forecast here; binary interval segment
serialization, bundle completion, source/topology documents, owner integration
and reader/viewer dispatch remain separate next steps. The fixed-width core
represents the current accepted interval channels, not a claim that future
connector/force-stage extensions cost no bytes.

Ownership is in this directory's CMakeLists. Standalone host qualification:

```sh
cmake -S output/full_shell -B <build> -DChrono_DIR=<existing-install>/lib/cmake/Chrono
cmake --build <build> --target robo_dyna_full_shell_record_check --parallel 1
ctest --test-dir <build> --parallel 1 --output-on-failure
```

The main app also exposes `ROBO_DYNA_ENABLE_FULL_SHELL_RECORD_CHECKS=ON`.
Tests cover canonical Python writer→C++→Python reader interoperability, exact
binary representations, bounded paths/arrays, rehashed semantic/phase failures,
pre-write rejection, preserved caller state, exact budget boundaries, and a
synthetic actual-count complete frame roundtrip. No CUDA/VSG tests run here.
