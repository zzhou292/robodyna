# Captured native projection replay

The adapter calls the already-retained private QE_Q1_CZCORP5 and QE_Q1_CZPROJN
routines in `qeph_q1_native`. No donor body or independent matrix implementation
is duplicated. Include Native.cmake after adding qualification/native/qeph; it
exposes `qeph_projection_replay`, linking the original target and the generated
CapturedRows fixture. The added Fortran wrapper inherits the original target's
MVSIZ129, precise flags, private modules and COMMON identifiers. C++ calls use
its existing NativeContext mutex.

Raw packets are per element; local IXC1..4 maps the actual captured row's four
world-omega values, which remain bound to its recorded original node indices.
The source routines have independent per-row algebra under the admitted explicit
NPT3/IDRIL0/IFINI0/IRESP0/IMPL_S0/IKPROJ0 controls. No physical source-key or
accepted-state authority is supplied by this qualification mapping.

Packing is explicit: geometry29 values, rate17, worldomega12, controls6, rate
result48 and force result24. Geometry and public VQ use row-major3x3; native
Fortran arrays are filled component-by-component. Force VF/VM retain the original
symmetric/antisymmetric slots. Positive area/inverse-area/LL/L13/L24 and finite
consumed operands are admitted before entering Fortran.

The fixture decoder authenticates the exact60604-byte captured JSONL, retains
cycles0/473/700 and all four original rows, and converts only array layout.
No input matrix or rate is reconstructed by production code. Undefined planar
DI/DB/VQN channels remain tagged undefined; their API storage is deterministically
zero and is never presented as an observed native numerical value. Fullcaller
X/V/VR/control data remain distinct from raw projection entry/exit data.

Root/lead own the actual numerical comparison and virtual-power tests. Source
preparation alone is not a replay pass, production-unit correction, or physical
accepted-step qualification. No production source is changed here.
