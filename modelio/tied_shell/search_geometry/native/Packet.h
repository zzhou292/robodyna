// SPDX-License-Identifier: MIT
#pragma once

// Test-only supplied-context ABI; never linked into the production adapter.
// Counts are by value. All arrays are contiguous. Connectivity and incidence
// order use 1-based local native indices, not original source IDs. q_nodes has
// four entries per Q4; t_nodes has three entries per T3. q_order and t_order
// are permutations of their family-local indices. Scalar field arrays contain
// Q4 entries followed by T3 entries. Lengths/thicknesses use the same working
// units; young_modulus is the native PM(20) ranking value in those units.
// Limits: 1 <= node_count <= 4096; 1 <= quad_count+triangle_count <= 128.
// The ordinary property branch uses IGEO(11)=1, IGEO(98)=0, NTY=2.
// selected={NELC,NELTG}; each is a family-local index or zero, independently.
// consumed={I2BUC1 master thickness,I2COR3 master thickness}.
// status=0 succeeds; status=1 rejects the packet without changing either
// result array. A no-match native result is a successful {0,0}/{0,0} packet.
// All pointers must be nonnull (an unused one-entry buffer suffices for an
// empty family); result arrays/status must be disjoint from each other and
// from input storage. The caller owns these test buffers through the call.
// Calls are serial: the independent native routine uses module context.
extern "C" void native_tied_search_geometry(
    int node_count, int quad_count, int triangle_count,
    const int* master_nodes, const int* q_nodes, const int* t_nodes,
    const int* q_order, const int* t_order,
    const double* geo_thickness, const double* young_modulus,
    const double* part_thickness, const double* element_thickness,
    int* selected, double* consumed, int* status);
