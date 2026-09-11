# Native shell association and consumed thickness packet

This test-only packet calls complete pinned `INCOQ3`; it does not call the
production match reducer or reproduce its comparison formula in a wrapper.
`Packet.h` documents the C ABI. C++ tests supply local node identities, complete
candidate layers and explicit incidence permutations, then compare native
results with the production value adapter. No full-source ordering registry,
original source admission, solid association, or mechanics is implied.

The packet retains each layer's GEO thickness, PM(20) modulus, part override
and element THK in distinct native slots. Q4 entries precede T3 entries in the
field arrays. The native T3 THK offset is therefore `NUMELC+NELTG`. Node
incidence is built from physical node occurrences in the supplied family
order; it does not sort or rank candidates. Empty Q4 or T3 populations are
allowed, and all borrowed counts/indices/permutations/values are checked
before the native call. The bounded packet has at most 4,096 local nodes and
128 total candidate layers, so all full-source triples can be tested through
separate small calls. It is not a full-vehicle array interface.

The first qualified context is ordinary property `IGEO(11)=1`, no layered
material override (`IGEO(98)=0`), `NTY=2`, `IINTTHICK=0`. Moduli are finite and
nonnegative; GEO thickness is finite positive. Part/element overrides are
finite nonnegative, with zero retaining native absence semantics. The complete
donor retains its other branches, but this packet does not admit them. Its
leading dimensions cover every referenced field; they are storage dimensions
for this test context, not production solver defaults.

Native source identities use OpenRadioss revision
`a62b27e6baa555d222a580d6218867d0be4d70b5`. All six retained files, including
the complete shared constant module and binary64 `my_real.inc`, have exact
byte counts, SHA256 and Git blob identities in `source-manifest.json`.

- `INCOQ3:81–125`: T3 lookup and native `ST>=STM` tie branch.
- `INCOQ3:128–162`: Q4 lookup and native `ST>STM` tie branch.
- `I2BUC1:214–234`: consumed master thickness, Q4 preferred if both indices
  remain nonzero; part override, then THK, then GEO.
- `I2COR3:152–171`: consumed master thickness with T3 preferred instead.

The two `.inc` files are exact byte slices of the retained callers. Each
wrapper adds only the closing `ENDIF` before the explicitly excluded solid
`ELSEIF`. `THKMAIN=ZERO` is caller initialization from `I2COR3:115`.
`NELC` and `NELTG` are returned independently; neither is inferred from a
single winner. Bounds and projection consumed thicknesses are also separate.
Secondary-node incidence is outside this packet; the first source adapter
separately requires the authenticated absence of secondary shell incidence.

`include/implicit_f.inc` replaces only native implicit/common context with
explicit module dimensions/counts and the complete native constants/type
header. The complete `INCOQ3` file is compiled unchanged. Native global context
makes calls serial. Result arrays are assigned only after the native call and
finite/range checks; malformed packet rejection leaves them unchanged.

## Owning build integration

The source adapter owns the CMake target and C++ tests outside this directory.
Compile these files in one test-only Fortran static library:

- `Context.F90`, `Interfaces.F90`, `Packet.F90`, `ConsumedThickness.F`
- `original/common_source/modules/constant_mod.F`
- `original/starter/source/interfaces/inter3d1/incoq3.F`

Include this directory, `include/`, and `original/engine/share/r8/`.
Use a dedicated `Fortran_MODULE_DIRECTORY` and flags `-cpp`,
`-ffixed-line-length-none`, `-ffree-line-length-none`, `-fno-fast-math`,
`-ffp-contract=off`, `-fcheck=all`, `-finit-real=snan`.
Register the existing shared verifier through:

```
python3 -B native/verify_sources.py /absolute/path/to/Total-Lagrangian-FEA
```

Author checks cover source identity, C++ header syntax and preprocessing only.
The owning native compiler/runtime and actual-source gates belong to root.
