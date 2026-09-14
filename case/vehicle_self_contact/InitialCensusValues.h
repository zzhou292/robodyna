#pragma once

#include "lib_src/collision/SelfContactBroadphaseTypes.h"
#include <cstddef>
#include <cstdint>

namespace crash::cases::vehicle_self_contact {

// One exact active-use row, reached through the separately supplied S0-to-active
// permutation. Rigid-group and canonical-vertex values are descriptive static
// source facts only; they do not admit or exclude a force event.
struct InitialCensusParentRow {
    std::uint64_t source_parent_id = 0;
    std::uint32_t surface_parent = UINT32_MAX;
    std::uint32_t facet_count = 0;
    std::uint32_t complete_rigid_group = UINT32_MAX;
    std::uint32_t vertices[4]{UINT32_MAX, UINT32_MAX,
                              UINT32_MAX, UINT32_MAX};
    std::uint8_t arity = 0;
};

enum class InitialCensusValueStatus {
    Ok,
    InvalidInput,
    IdentityMismatch,
    InvalidPairKey,
    Unrepresentable
};

struct InitialFacetCapacityCensus {
    std::size_t current_inflated_aabb_overlap_parent_pairs = 0;
    std::size_t q4_q4_parent_pairs = 0;
    std::size_t q4_t3_parent_pairs = 0;
    std::size_t t3_t3_parent_pairs = 0;
    std::size_t level0_facet_pairs = 0;
    std::size_t same_complete_rigid_group_parent_pairs = 0;
    std::size_t shared_canonical_vertex_parent_pairs = 0;
    std::size_t shared_canonical_edge_parent_pairs = 0;
    std::uint64_t pair_key_hash = 0;
    std::uint64_t surface_active_source_hash = 0;
    bool sorted_unique_canonical_keys = false;
    bool no_self_or_reversed_pair_keys = false;
    bool descriptive_static_policy_only = false;
    bool feature_discovery_performed = false;
    bool intersection_processing_performed = false;
    bool force_admission_performed = false;
};

struct InitialCensusValueReport {
    InitialCensusValueStatus status = InitialCensusValueStatus::Ok;
    const char* message = "OK";
    std::size_t parent = SIZE_MAX;
    std::size_t pair = SIZE_MAX;
};

// Allocation-free checked census over one complete broadphase readback. The
// surface_to_active map must be a complete permutation authenticated by each
// active row's reverse surface_parent. Successful output is published at once.
InitialCensusValueReport CountInitialFacetCapacity(
    const tlfea::contact::SelfContactPairKey* keys, std::size_t key_count,
    const std::uint32_t* surface_to_active, std::size_t surface_parent_count,
    const InitialCensusParentRow* active_parents,
    std::size_t active_parent_count,
    InitialFacetCapacityCensus* output) noexcept;

}  // namespace crash::cases::vehicle_self_contact
