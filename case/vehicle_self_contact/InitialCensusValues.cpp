#include "InitialCensusValues.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace crash::cases::vehicle_self_contact {
namespace {

using Key = tlfea::contact::SelfContactPairKey;
using Status = InitialCensusValueStatus;

InitialCensusValueReport Failure(Status status, const char* message,
                                 std::size_t parent = SIZE_MAX,
                                 std::size_t pair = SIZE_MAX) noexcept {
    return {status, message, parent, pair};
}

bool Add(std::size_t value, std::size_t& total) noexcept {
    if (value > std::numeric_limits<std::size_t>::max() - total)
        return false;
    total += value;
    return true;
}

bool Multiply(std::size_t first, std::size_t second,
              std::size_t& output) noexcept {
    if (first && second >
            std::numeric_limits<std::size_t>::max() / first)
        return false;
    output = first * second;
    return true;
}

void Hash(std::uint64_t value, std::uint64_t& hash) noexcept {
    for (unsigned byte = 0; byte < 8; ++byte) {
        hash ^= (value >> (8 * byte)) & 0xffu;
        hash *= 1099511628211ULL;
    }
}

bool ValidParent(const InitialCensusParentRow& parent) noexcept {
    if (!parent.source_parent_id ||
        !((parent.arity == 4 && parent.facet_count == 2) ||
          (parent.arity == 3 && parent.facet_count == 1)))
        return false;
    for (unsigned i = 0; i < parent.arity; ++i) {
        if (parent.vertices[i] == UINT32_MAX) return false;
        for (unsigned j = 0; j < i; ++j)
            if (parent.vertices[i] == parent.vertices[j]) return false;
    }
    return true;
}

unsigned SharedVertices(const InitialCensusParentRow& first,
                        const InitialCensusParentRow& second) noexcept {
    unsigned count = 0;
    for (unsigned a = 0; a < first.arity; ++a)
        for (unsigned b = 0; b < second.arity; ++b)
            count += first.vertices[a] == second.vertices[b];
    return count;
}

bool SharedEdge(const InitialCensusParentRow& first,
                const InitialCensusParentRow& second) noexcept {
    for (unsigned a = 0; a < first.arity; ++a) {
        const auto a0 = first.vertices[a];
        const auto a1 = first.vertices[(a + 1) % first.arity];
        const auto alo = std::min(a0, a1);
        const auto ahi = std::max(a0, a1);
        for (unsigned b = 0; b < second.arity; ++b) {
            const auto b0 = second.vertices[b];
            const auto b1 = second.vertices[(b + 1) % second.arity];
            if (alo == std::min(b0, b1) &&
                ahi == std::max(b0, b1))
                return true;
        }
    }
    return false;
}

}  // namespace

InitialCensusValueReport CountInitialFacetCapacity(
    const Key* keys, std::size_t key_count,
    const std::uint32_t* surface_to_active,
    std::size_t surface_parent_count,
    const InitialCensusParentRow* active_parents,
    std::size_t active_parent_count,
    InitialFacetCapacityCensus* output) noexcept {
    if (!output || (key_count && !keys) || !surface_to_active ||
        !active_parents || !surface_parent_count ||
        surface_parent_count != active_parent_count ||
        surface_parent_count > UINT32_MAX)
        return Failure(Status::InvalidInput,
            "Initial census storage or parent extent is invalid");

    InitialFacetCapacityCensus next;
    next.pair_key_hash = 14695981039346656037ULL;
    next.surface_active_source_hash = 14695981039346656037ULL;
    Hash(surface_parent_count, next.surface_active_source_hash);
    for (std::size_t active = 0; active < active_parent_count; ++active) {
        const auto& parent = active_parents[active];
        if (!ValidParent(parent) ||
            parent.surface_parent >= surface_parent_count ||
            surface_to_active[parent.surface_parent] != active)
            return Failure(Status::IdentityMismatch,
                "Active-use row does not reverse the complete S0 mapping",
                active);
    }
    for (std::size_t surface = 0; surface < surface_parent_count; ++surface) {
        const auto active = surface_to_active[surface];
        if (active >= active_parent_count ||
            active_parents[active].surface_parent != surface)
            return Failure(Status::IdentityMismatch,
                "S0 parent does not map to one exact active-use parent",
                surface);
        const auto& parent = active_parents[active];
        Hash(surface, next.surface_active_source_hash);
        Hash(active, next.surface_active_source_hash);
        Hash(parent.source_parent_id, next.surface_active_source_hash);
        Hash(parent.arity, next.surface_active_source_hash);
        Hash(parent.facet_count, next.surface_active_source_hash);
        Hash(parent.complete_rigid_group,
             next.surface_active_source_hash);
        for (unsigned local = 0; local < parent.arity; ++local)
            Hash(parent.vertices[local],
                 next.surface_active_source_hash);
    }

    Hash(key_count, next.pair_key_hash);
    Key previous = 0;
    for (std::size_t pair = 0; pair < key_count; ++pair) {
        const auto key = keys[pair];
        const auto first_surface =
            tlfea::contact::FirstSurfaceParent(key);
        const auto second_surface =
            tlfea::contact::SecondSurfaceParent(key);
        if ((pair && key <= previous) ||
            first_surface >= second_surface ||
            second_surface >= surface_parent_count)
            return Failure(Status::InvalidPairKey,
                "Broadphase keys are not sorted unique canonical S0 pairs",
                SIZE_MAX, pair);
        previous = key;
        Hash(key, next.pair_key_hash);
        const auto first_active = surface_to_active[first_surface];
        const auto second_active = surface_to_active[second_surface];
        if (first_active >= active_parent_count ||
            second_active >= active_parent_count ||
            first_active == second_active)
            return Failure(Status::IdentityMismatch,
                "Broadphase pair does not map to distinct active-use parents",
                SIZE_MAX, pair);
        const auto& first = active_parents[first_active];
        const auto& second = active_parents[second_active];
        std::size_t* pair_class = nullptr;
        if (first.facet_count == 2 && second.facet_count == 2)
            pair_class = &next.q4_q4_parent_pairs;
        else if (first.facet_count == 1 && second.facet_count == 1)
            pair_class = &next.t3_t3_parent_pairs;
        else
            pair_class = &next.q4_t3_parent_pairs;
        std::size_t facet_pairs = 0;
        if (!Multiply(first.facet_count, second.facet_count,
                      facet_pairs) ||
            !Add(1, *pair_class) ||
            !Add(facet_pairs, next.level0_facet_pairs))
            return Failure(Status::Unrepresentable,
                "Initial level0 facet-pair expansion overflows",
                SIZE_MAX, pair);
        if (first.complete_rigid_group != UINT32_MAX &&
            first.complete_rigid_group ==
                second.complete_rigid_group &&
            !Add(1,
                next.same_complete_rigid_group_parent_pairs))
            return Failure(Status::Unrepresentable,
                "Initial same-rigid pair count overflows",
                SIZE_MAX, pair);
        if (SharedVertices(first, second) &&
            !Add(1,
                next.shared_canonical_vertex_parent_pairs))
            return Failure(Status::Unrepresentable,
                "Initial shared-vertex pair count overflows",
                SIZE_MAX, pair);
        if (SharedEdge(first, second) &&
            !Add(1,
                next.shared_canonical_edge_parent_pairs))
            return Failure(Status::Unrepresentable,
                "Initial shared-edge pair count overflows",
                SIZE_MAX, pair);
    }
    std::size_t classified = 0;
    if (!Add(next.q4_q4_parent_pairs, classified) ||
        !Add(next.q4_t3_parent_pairs, classified) ||
        !Add(next.t3_t3_parent_pairs, classified) ||
        classified != key_count)
        return Failure(Status::Unrepresentable,
            "Initial parent-pair classes do not cover the pair set");
    next.current_inflated_aabb_overlap_parent_pairs = key_count;
    next.sorted_unique_canonical_keys = true;
    next.no_self_or_reversed_pair_keys = true;
    next.descriptive_static_policy_only = true;
    // Exact feature/intersection processing is deliberately deferred until
    // transaction capacities have been selected from this census.
    next.feature_discovery_performed = false;
    next.intersection_processing_performed = false;
    next.force_admission_performed = false;
    *output = next;
    return {};
}

InitialCensusValueReport CountInitialFacetFilterCensus(
    const Key* keys, std::size_t key_count,
    const std::uint32_t* surface_to_active,
    std::size_t surface_parent_count,
    const InitialCensusParentRow* active_parents,
    std::size_t active_parent_count,
    const tlfea::contact::CurrentFixedTriangle* triangles,
    std::size_t triangle_count, std::uint64_t source_identity_hash,
    tlfea::contact::FixedTrianglePair* exact_sample,
    std::size_t exact_sample_capacity,
    InitialFacetFilterCensus* output) noexcept {
    if (!output || (key_count && !keys) || !surface_to_active ||
        !active_parents || !triangles || !surface_parent_count ||
        surface_parent_count != active_parent_count ||
        surface_parent_count > UINT32_MAX || !triangle_count ||
        !source_identity_hash ||
        (exact_sample_capacity && !exact_sample))
        return Failure(Status::InvalidInput,
            "Initial filter census storage or extent is invalid");

    std::size_t next_facet = 0;
    for (std::size_t active = 0; active < active_parent_count; ++active) {
        const auto& parent = active_parents[active];
        if (!ValidParent(parent) ||
            parent.surface_parent >= surface_parent_count ||
            surface_to_active[parent.surface_parent] != active ||
            parent.facet_offset != next_facet ||
            parent.facet_count > triangle_count - next_facet ||
            !std::isfinite(parent.reference_half_thickness_m) ||
            !(parent.reference_half_thickness_m > 0))
            return Failure(Status::IdentityMismatch,
                "Filter parent row does not match the complete facet roster",
                active);
        next_facet += parent.facet_count;
    }
    if (next_facet != triangle_count)
        return Failure(Status::IdentityMismatch,
            "Filter parent rows do not cover accepted represented geometry");
    for (std::size_t surface = 0; surface < surface_parent_count; ++surface) {
        const auto active = surface_to_active[surface];
        if (active >= active_parent_count ||
            active_parents[active].surface_parent != surface)
            return Failure(Status::IdentityMismatch,
                "Filter S0 map is not one complete permutation", surface);
    }

    InitialFacetFilterCensus next;
    next.category_hash = 14695981039346656037ULL;
    next.exact_sample_hash = 14695981039346656037ULL;
    next.source_identity_hash = source_identity_hash;
    Hash(source_identity_hash, next.category_hash);
    Hash(key_count, next.category_hash);
    Hash(triangle_count, next.category_hash);
    Hash(source_identity_hash, next.exact_sample_hash);
    Hash(triangle_count, next.exact_sample_hash);
    Key previous = 0;
    for (std::size_t pair = 0; pair < key_count; ++pair) {
        const auto key = keys[pair];
        const auto first_surface =
            tlfea::contact::FirstSurfaceParent(key);
        const auto second_surface =
            tlfea::contact::SecondSurfaceParent(key);
        if ((pair && key <= previous) ||
            first_surface >= second_surface ||
            second_surface >= surface_parent_count)
            return Failure(Status::InvalidPairKey,
                "Filter keys are not sorted unique canonical S0 pairs",
                SIZE_MAX, pair);
        previous = key;
        const auto first_active = surface_to_active[first_surface];
        const auto second_active = surface_to_active[second_surface];
        if (first_active >= active_parent_count ||
            second_active >= active_parent_count ||
            first_active == second_active)
            return Failure(Status::IdentityMismatch,
                "Filter pair cannot map to two active-use parents",
                SIZE_MAX, pair);
        const auto& first = active_parents[first_active];
        const auto& second = active_parents[second_active];
        for (std::size_t first_local = 0;
             first_local < first.facet_count; ++first_local) {
            const auto first_facet = first.facet_offset + first_local;
            for (std::size_t second_local = 0;
                 second_local < second.facet_count; ++second_local) {
                const auto second_facet =
                    second.facet_offset + second_local;
                const auto filtered =
                    tlfea::contact::ClassifyAcceptedFacetPair(
                        triangles[first_facet],
                        first.reference_half_thickness_m,
                        first.complete_rigid_group,
                        triangles[second_facet],
                        second.reference_half_thickness_m,
                        second.complete_rigid_group);
                if (filtered.status !=
                    tlfea::contact::SelfContactFacetFilterStatus::Ok)
                    return Failure(Status::IdentityMismatch,
                        "Production facet filter rejected represented geometry",
                        SIZE_MAX, pair);
                std::size_t* category = nullptr;
                using Category =
                    tlfea::contact::SelfContactFacetFilterCategory;
                switch (filtered.category) {
                  case Category::ExcludedSameRigidGroup:
                    category = &next.excluded_same_rigid_group;
                    break;
                  case Category::CoordinateAabbSeparated:
                    category = &next.coordinate_aabb_separated;
                    break;
                  case Category::FaceAxisSeparated:
                    category = &next.face_axis_separated;
                    break;
                  case Category::EdgeCrossAxisSeparated:
                    category = &next.edge_cross_axis_separated;
                    break;
                  case Category::VertexEdgeAxisSeparated:
                    category = &next.vertex_edge_axis_separated;
                    break;
                  case Category::VertexVertexAxisSeparated:
                    category = &next.vertex_vertex_axis_separated;
                    break;
                  case Category::ExactRemaining:
                    category = &next.exact_remaining;
                    if (next.exact_sample_count <
                            exact_sample_capacity) {
                        if (first_facet > UINT32_MAX ||
                            second_facet > UINT32_MAX)
                            return Failure(Status::Unrepresentable,
                                "Exact sample facet index is unrepresentable",
                                SIZE_MAX, pair);
                        exact_sample[next.exact_sample_count++] = {
                            static_cast<std::uint32_t>(first_facet),
                            static_cast<std::uint32_t>(second_facet)};
                        Hash(first_facet, next.exact_sample_hash);
                        Hash(second_facet, next.exact_sample_hash);
                    }
                    break;
                }
                if (!category || !Add(1, *category) ||
                    !Add(1, next.represented_facet_pairs))
                    return Failure(Status::Unrepresentable,
                        "Initial filter category count overflows",
                        SIZE_MAX, pair);
                Hash(key, next.category_hash);
                Hash(first_facet, next.category_hash);
                Hash(second_facet, next.category_hash);
                Hash(static_cast<std::uint8_t>(filtered.category),
                     next.category_hash);
            }
        }
    }
    std::size_t accounted = 0;
    if (!Add(next.excluded_same_rigid_group, accounted) ||
        !Add(next.coordinate_aabb_separated, accounted) ||
        !Add(next.face_axis_separated, accounted) ||
        !Add(next.edge_cross_axis_separated, accounted) ||
        !Add(next.vertex_edge_axis_separated, accounted) ||
        !Add(next.vertex_vertex_axis_separated, accounted) ||
        !Add(next.exact_remaining, accounted) ||
        accounted != next.represented_facet_pairs)
        return Failure(Status::Unrepresentable,
            "Initial filter categories do not partition facet pairs");
    Hash(next.exact_sample_count, next.exact_sample_hash);
    next.complete_disjoint_accounting = true;
    next.production_certificates_used = true;
    next.feature_discovery_performed = false;
    next.interval_crossing_performed = false;
    *output = next;
    return {};
}

}  // namespace crash::cases::vehicle_self_contact
