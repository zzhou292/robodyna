# Beam18 reference qualification

Source pin: OpenRadioss a62b27e6baa555d222a580d6218867d0be4d70b5.

The native packet compiles complete PCOORI/PEVECI/PMASS and the complete
DEFBEAM_SECT subroutine from the authenticated full HM_READ_PROP18. Exact
HM_READ_MAT44 shear preparation and HM_READ_PROP18 zero-df resolution are
selected verbatim. Private context fixes one parent, three source nodes,
MVSIZ1/NPROPG512/NPROPM100/NPROPGI32, thermal off and I7STIFS1. It records
message errors, all section values, SKEW, MSP/INP, actual nodal STI/STIR and
PARTSAV mass. N3 nodal stiffness remains zero. These packet dimensions are
qualification bounds, not native global model dimensions.

The API does not expose PINIT3's later DT1LAWP or PIBUF3 results. It does not
claim a startup timestep, OFF/history initialization, physical/added inertia
partition or global M/J scatter. The source-reader release/offset policy is
explicit and narrower than the general native beam family.

Root gate:

```
cmake -S lib_utest/qualification/beam18_reference -B <build> \
  -DCMAKE_BUILD_TYPE=Release -DBEAM18_NATIVE=ON -DBEAM18_CUDA=ON \
  -DCMAKE_CUDA_ARCHITECTURES=120 \
  -DBEAM18_SOURCE_FIXTURE=<reports>/yaris-beam18-geometry-1
cmake --build <build> -j2
ctest --test-dir <build> --output-on-failure -j1
```

Default configuration runs only small host/source checks. The original fixture
is build-generated from fixed-hash binaries; neither TL production nor native
math depends on the app importer. The create-only exporter is a staging tool
that reuses existing app canonical and working-coordinate readers.

The optional N3 normalization in the C++ packet is bound to the complete
pinned HM_READ_BEAM reader and its exact lines158–183, emitted as
`reader_orientation_identity.inc` for identity checking only. The selected
node-defined profile has no vector override: absent or endpoint-alias N3
resolves to N2 before native system indexing. N3 is CHECK_USED, while only
N1/N2 are CHECK_BEAM. No reader behavior or numerical oracle changed.
