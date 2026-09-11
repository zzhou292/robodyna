# Complete physical startup

This first source-only increment resolves roles from the retained physical model
and CIN attachment model. It does not grant degrees of freedom or initialize a
CUDA owner. TYPE13 endpoint census includes only N1/N2; N3 is orientation data.

The original fixture gate is
`VehiclePhysicalAttachmentsOriginal.Type13ActualEndpointRigidAndCinRoleCensus`
in `robo_dyna_vehicle_physical_attachments_check`. It emits complete ordered
EID/endpoint/NID intersections with PART, plain rigid, CIN secondary and CIN
master roles as XML properties. No intersection count is inferred from material
or coordinate equality. The existing full-source fixture owns its setup.

Author qualification: both new translation units pass C++17 syntax checking
under 1 CPU/512 MiB (`vehicle-runtime-role-syntax-2.json`). The actual-source
census is deliberately root-owned and has not been run by the author. The first
syntax attempt used the wrong TYPE25 connection field name; only that field
access was corrected before the successful check. No mechanics changed.
