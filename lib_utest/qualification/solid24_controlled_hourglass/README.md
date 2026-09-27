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

This first gate is SI/metre only. It does not admit the vehicle's Millimetre
reference profile or claim full native working-unit floor parity. That requires
a separate full native-mm caller gate before widening admission.

End-to-end geometry/material and force stage comparisons reuse the existing
HEPH native contract (3e-10 group scale and its1e-20 floor), because the geometry
and nonlinear material evaluations precede this adapter. The independently
qualified raw controlled-hourglass leaf keeps all its128-epsilon field bounds
unchanged. Signed modal work additionally requires exact own-field ordered
replay and the measured-modal-drift plus gamma13 arithmetic bound.

Configure TL_H24_ADAPTER_CUDA=ON for actual-device tests and the existing IC0
CUDA regression. No-device execution fails instead of skipping.
