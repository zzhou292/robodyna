# Shared selected solid material caller values

`materials/detail/SolidCallerValues.h` extracts the qualified LAW36/solid18
SRHO3 density, MQVISCB active zero-material-viscosity branch, and no-EOS
MULAW/MMAIN internal-work arithmetic. It preserves expression ordering and
requires explicit native volume, density-length and sound-speed floors. The
legacy LAW36 entry supplies its original SI literals. Plastic work, material
history, returned sound speed, curve ownership and failure publication remain
with each material/element caller.

The CMake host target runs two independent density/hydrostatic/floor controls
plus the existing five LAW36 and four solid18 force host functions. Root must
also rerun the existing solid_law36_point and solid18_force native/CUDA gates.
No material-law admission or different floor for a legacy caller is implied.

The source manifest binds this extraction and the two affected call sites.
Their pre-extraction reference is TL f79e2ab; the existing native qualifiers
retain complete source donors and independent caller observations. This helper
introduces no Fortran substitute and no new standalone element core.
