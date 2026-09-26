# Accepted raw nodal mass view

This owner accessor exposes current accepted scalar MS in SI kg before this
attempt's CIN transfer. It borrows the existing accepted CIN slab, including
explicitly empty CIN; it has no allocation or alternate mass state. The pointer
and every identity field authenticate only in the exact open assembly. Stale,
foreign, sealed or discarded views must not be read.

The tests reuse the complete native CIN force/motion oracle for three committed
stages, with a real discarded/retried stage and no native coefficient reseeding.
Mass is compared before each transfer and after publication; zero secondary M
and transferred master M remain distinct from saved release history. Separate
cases cover true empty CIN/fixed positive raw M, absence of any raw store, and
disjoint PART/plain rigid members through three real owner commits. The new
view cannot replace member M by aggregate/generalized mass or reconstruct M
from inverse mass. Device allocations remain unchanged.

The accessor performs no unit conversion. A future native contact consumer
must divide these SI coefficients by its explicit native mass scale, admitting
genuine zero and rejecting nonfinite or positive-underflow conversion. The
existing static TYPE25 mass cache and rigid/CIN admission guards are unchanged.
These are accessor/owner/native phase tests, not a coupled contact scene or
vehicle profile/performance qualification.

Configure with explicit gfortran-local/nvcc/architecture120 and
`TL_CIN_NATIVE_CACHE` naming the pinned `openradioss-tied-interface-1` cache.
Run CTest excluding `cuda` under the host guard, then only `cuda` under the
actual owning GPU guard. The production consumer links no native oracle or
GTest. Parent CIN host/native/source and actual CUDA regressions are included.
