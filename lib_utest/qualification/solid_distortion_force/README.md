# Native eight-node distortion force

Complete element-local S8FOR_DISTOR response, source-isolated from all active
profiles. Small headers retain native damping, face classification, projection,
penalty, center/corner order, force scatter, STI and cumulative EINT_DISTOR.
Direct signed work is accumulated separately, so a large carried energy cannot
hide a nonzero increment. No floating-point force atomics or global contact path.

`Force.h` is the raw native-number core (`distortion::native`). `UnitResponse.h`
is the SI boundary: PrepareForceValues(material, parameters, forceInput, units,
prepared), ClassifyDamping(prepared, activity), then EvaluateForce(prepared,
native_batch_damping_enabled, result). UnitScale is explicitly SI={1,1,1} or
mm/Mg/s={.001,1000,1}, reusing existing Radioss unit-factor arithmetic. Geometry,
material slots and damping parameters are evaluated in selected native numbers.
The old c031 SI-only parameter leaf remains unchanged. Native FLD*velocity is
working-unit-dependent; no missing-length dimensional correction is inserted.

The batch Boolean is required. It is the integer OR of triggers_native_batch over
the original native NEL packet, not a CUDA block or arbitrary material grouping.
A zero-mean buckled row cannot trigger that OR but is damped when another row
triggers it. Native geometry still runs when damping does not. Native slot order
must be preserved through all face maps; map to source nodes only afterward.

The test bridge only packs arrays into the independently qualified full a62
S8FOR_DISTOR. It reuses native SDISTOR parameters and numerical leaves from the
94fc2db3 oracle. SI and original-mm responses compare all 24 force components,
STI, cumulative energy, direct work, and positive center/corner contact counts.
The two-row native batch witness uses regular geometry because the existing
oracle's passive geometry observer intentionally admits one-row contacts only.
No native numerical statement is changed. This is not a full H24 family caller,
owner integration, a source-matched vehicle run, or a CPU/GPU speed ratio.
