# LAW90 controlled distortion caller

This gate composes the existing eight-point LAW90 force path with the shared native distortion implementation. It does not enable a vehicle or introduce a solver clock.

`Material` prepares native mechanical slots once. `Reference` caches the original SI reference and the native-number reference. `PrepareInitial` and `PrepareCandidate` run the existing geometry/material body in the selected working units and expose a damping trigger. The resident owner must reduce that integer trigger over the authenticated native NEL packet before calling `Complete` for each row. A local one-row decision is valid only for the one-row test packet.

History remains in native working units between steps. Force, timestep, summed nodal stiffness, material work and distortion energy are converted once at the output boundary. Native final-IP STI is retained as an observation; SCUMU3 uses the independently accumulated STIN. No hourglass contribution exists in this fully integrated family.

The native oracle carries its own material history, computes all eight Gauss points, obtains its own orientation, invokes SDISTOR_INI and S8FOR_DISTOR, and converts final outputs. Tests cover both units, reflected source ordering,32-step recurrence, native curve relocation and failed-input publication. The actual radiator has an explicit15MPa cutoff; ambiguous resolved default cutoff sentinels are rejected in the native-mm profile.

Host/source and actual CUDA execution must pass before resident integration. Existing vehicle qualification and timing remain separate.
