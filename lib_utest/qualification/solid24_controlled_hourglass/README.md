# HEPH24 controlled stage before distortion

Independent complete a62b27e6 native execution supplies geometry, material,
controlled FHOUR/work/STI, local HG force, local SFINT force, and world force
before distortion. The native caller still runs distortion and SCUMU3, with
RequiredStages255. Reversible output-only hooks expose intermediate values;
no legacy or final-force subtraction produces an expected result.

The production stage reuses the existing HEPH kinematics, LAW42 caller binding,
SFINT3 and ordered SRROTA/source mapping. Controlled history has force units and
is not interchangeable with legacy Pa history. No public owner/profile is enabled.
The result explicitly names force/STI before distortion and includes native-slot
world values for the later eight-node distortion leaf.

The direct BeforeDistortion value stage admits metre numerical operands only.
The explicit UnitResponse boundary also implements the actual mm/Mg/s profile:
it prepares private native-numeric reference/material values, runs the same
geometry/material/HG source expressions with native literal floors, and converts
all output dimensions back to SI. It reuses the existing contact UnitScale and
UnitFactors also used by distortion; no new dimensional convention is inferred.
The original physical reference/unit stamp is retained. Unsupported unit triples
and stamp mismatches reject atomically. Public owner/profile enable remains gated.

Working-unit tests execute the complete native H24 oracle in independent raw
mm/Mg/s input values, then convert each labelled output channel. A tiny cell
above/below the native1e-20mm3 reference/material floor distinguishes this from
incorrect SI evaluation followed by relabelling. Carried work additionally uses
a native-numeric modal witness: exact replay happens before unit conversion,
so conversion rounding cannot masquerade as a work-order defect.

End-to-end geometry/material and force stage comparisons reuse the existing
HEPH native contract (3e-10 group scale and its1e-20 floor), because the geometry
and nonlinear material evaluations precede this adapter. The independently
qualified raw controlled-hourglass leaf keeps all its128-epsilon field bounds
unchanged. Signed modal work additionally requires exact own-field ordered
replay and the measured-modal-drift plus gamma13 arithmetic bound.

Configure TL_H24_ADAPTER_CUDA=ON for actual-device tests and the existing IC0
CUDA regression. No-device execution fails instead of skipping.
