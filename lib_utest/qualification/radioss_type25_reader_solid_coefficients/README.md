# Signed reader and internal solid main coefficients

This additive EightSlot value API reproduces I25GAPM before shell replacement.
The first support uses its actual ICONTR and PM32/PM107; the second support uses
PM32, then the original mean and internal-face sign. Both signed nonzero raw
reader volumes survive. The existing positive exterior API is unchanged.

Membership, pre-shell support ordering, raw geometry and reader phase belong to
the source layer. This API neither chooses supports nor enables mixed runtime.
The oracle extracts the original complete selected branch and VOLINT from the
existing pinned OpenRadioss sources. Production does not link either oracle.
Host and CUDA tests cover asymmetric coefficients, signed volumes, first-control
selection, signed zero, unchanged legacy admission, invalid atomic outputs and
retry. CUDA tests vary launch order and size.
