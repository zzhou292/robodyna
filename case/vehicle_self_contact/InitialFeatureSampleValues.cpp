#include "InitialFeatureSampleValues.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <limits>
#include <type_traits>

namespace crash::cases::vehicle_self_contact {
namespace {

namespace contact = tlfea::contact;

constexpr std::uint64_t HashBasis = 14695981039346656037ULL;
constexpr std::uint64_t HashPrime = 1099511628211ULL;

bool Same(const contact::FixedTriangleKey& first,
          const contact::FixedTriangleKey& second) noexcept {
    return first.source_instance_id == second.source_instance_id &&
        first.parent_eid == second.parent_eid &&
        first.level == second.level &&
        first.local_facet == second.local_facet;
}

template <class T,
          std::enable_if_t<std::is_integral_v<T>, int> = 0>
void Hash(T value, std::uint64_t& hash) noexcept {
    const auto bits = static_cast<std::uint64_t>(value);
    for (unsigned byte = 0; byte < 8; ++byte) {
        hash ^= (bits >> (8 * byte)) & 0xffu;
        hash *= HashPrime;
    }
}

void Hash(double value, std::uint64_t& hash) noexcept {
    std::uint64_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(bits));
    Hash(bits, hash);
}

void Hash(const contact::FixedTriangleKey& key,
          std::uint64_t& hash) noexcept {
    Hash(key.source_instance_id, hash);
    Hash(key.parent_eid, hash);
    Hash(key.level, hash);
    Hash(key.local_facet, hash);
}

void Hash(const contact::FacetVertexKey& key,
          std::uint64_t& hash) noexcept {
    Hash(key.source_instance_id, hash);
    Hash(static_cast<std::uint8_t>(key.kind), hash);
    Hash(key.first, hash);
    Hash(key.second, hash);
    Hash(key.numerator, hash);
    Hash(key.denominator, hash);
    Hash(key.level, hash);
    Hash(key.grid_i, hash);
    Hash(key.grid_j, hash);
}

void Hash(const contact::FacetEdgeKey& key,
          std::uint64_t& hash) noexcept {
    Hash(key.parent_boundary, hash);
    Hash(key.parent_eid, hash);
    Hash(key.endpoints[0], hash);
    Hash(key.endpoints[1], hash);
}

void Hash(const contact::FixedTriangleStratumKey& key,
          std::uint64_t& hash) noexcept {
    Hash(static_cast<std::uint8_t>(key.kind), hash);
    if (key.kind == contact::FixedTriangleStratumKind::Vertex)
        Hash(key.vertex, hash);
    else if (key.kind == contact::FixedTriangleStratumKind::Edge)
        Hash(key.edge, hash);
    else
        Hash(key.face, hash);
}

void Hash(const contact::FixedTriangleFeatureKey& key,
          std::uint64_t& hash) noexcept {
    Hash(static_cast<std::uint8_t>(key.kind), hash);
    if (key.kind == contact::FixedTriangleCandidateKind::VertexFace) {
        Hash(key.vertex_face.vertex, hash);
        Hash(key.vertex_face.target, hash);
    } else {
        Hash(key.edge_edge.edges[0], hash);
        Hash(key.edge_edge.edges[1], hash);
    }
}

void Hash(contact::Vec3 value, std::uint64_t& hash) noexcept {
    Hash(value.x, hash);
    Hash(value.y, hash);
    Hash(value.z, hash);
}

void Hash(const contact::FixedTriangleFeatureCandidate& value,
          std::uint64_t& hash) noexcept {
    Hash(value.key, hash);
    for (unsigned side = 0; side < 2; ++side) {
        Hash(value.triangles[side], hash);
        Hash(value.local_features[side], hash);
        Hash(value.points[side], hash);
        Hash(value.edge_parameters[side], hash);
    }
    for (double weight : value.face_weights)
        Hash(weight, hash);
    Hash(value.distance_m, hash);
    Hash(value.representation_error_m, hash);
}

void Hash(const contact::FixedTriangleIntersection& value,
          std::uint64_t& hash) noexcept {
    Hash(value.triangles[0], hash);
    Hash(value.triangles[1], hash);
    Hash(static_cast<std::uint8_t>(value.kind), hash);
    Hash(static_cast<std::uint8_t>(value.local_exclusion), hash);
}

bool Add(std::size_t value, std::size_t& total) noexcept {
    if (value > std::numeric_limits<std::size_t>::max() - total)
        return false;
    total += value;
    return true;
}

bool Accumulate(
    const contact::FixedTriangleDiscoveryReport& report,
    InitialFeatureSampleIdentity& total) noexcept {
    return Add(report.potential_tasks, total.potential_tasks) &&
        Add(report.local_masked_tasks, total.local_masked_tasks) &&
        Add(report.exact_executed_tasks,
            total.exact_executed_tasks) &&
        Add(report.raw_feature_candidates,
            total.raw_feature_candidates) &&
        Add(report.feature_candidates, total.feature_candidates) &&
        Add(report.raw_intersections, total.raw_intersections) &&
        Add(report.intersections, total.intersections);
}

std::uint64_t Microseconds(
    std::chrono::steady_clock::duration duration) noexcept {
    const auto value =
        std::chrono::duration_cast<std::chrono::microseconds>(
            duration).count();
    return value > 0 ? static_cast<std::uint64_t>(value) : 0;
}

}  // namespace

contact::FixedTriangleFeatureLimits
InitialFeatureSampleDiscoveryLimits(
    std::size_t chunk_capacity,
    unsigned worker_count) noexcept {
    contact::FixedTriangleFeatureLimits result;
    if (!chunk_capacity ||
        chunk_capacity >
            std::numeric_limits<std::size_t>::max() / 15) {
        result.max_input_pairs = 0;
        return result;
    }
    result.max_input_pairs = chunk_capacity;
    result.max_triangle_references = 2 * chunk_capacity;
    result.max_vertex_references = 6 * chunk_capacity;
    result.max_edge_references = 6 * chunk_capacity;
    result.max_raw_feature_candidates = 15 * chunk_capacity;
    result.max_feature_candidates = 15 * chunk_capacity;
    result.max_raw_intersections = chunk_capacity;
    result.max_intersections = chunk_capacity;
    result.max_host_bytes = InitialExactFeatureHostByteCap;
    result.worker_count = worker_count;
    return result;
}

InitialFeatureSampleReport DiscoverInitialFeatureSample(
    contact::FixedTriangleFeatureDiscovery& discovery,
    const contact::CurrentFixedTriangle* triangles,
    std::size_t triangle_count,
    const contact::FixedTrianglePair* sample_pairs,
    std::size_t sample_pair_count,
    contact::FixedTriangleFeatureTaskMask* task_masks,
    std::size_t chunk_capacity,
    InitialFeatureSampleResult* output) noexcept {
    if (!output || !triangles || !triangle_count ||
        (sample_pair_count && !sample_pairs) || !task_masks ||
        !chunk_capacity)
        return {InitialFeatureSampleStatus::InvalidInput, SIZE_MAX,
                SIZE_MAX, contact::FixedTriangleArithmeticReason::None,
                "Initial exact feature sample storage is incomplete"};

    InitialFeatureSampleResult next;
    next.complete.feature_hash = HashBasis;
    next.complete.intersection_hash = HashBasis;
    for (std::size_t begin = 0; begin < sample_pair_count;
         begin += std::min(chunk_capacity, sample_pair_count - begin)) {
        const auto count =
            std::min(chunk_capacity, sample_pair_count - begin);
        const auto mask_started = std::chrono::steady_clock::now();
        for (std::size_t local = 0; local < count; ++local) {
            const auto& pair = sample_pairs[begin + local];
            if (pair.first >= triangle_count ||
                pair.second >= triangle_count) {
                return {InitialFeatureSampleStatus::InvalidInput,
                        begin + local, SIZE_MAX,
                        contact::FixedTriangleArithmeticReason::None,
                        "Initial exact sample pair is out of range"};
            }
            const auto status =
                contact::BuildFixedTriangleFeatureTaskMask(
                    triangles[pair.first], triangles[pair.second],
                    task_masks + local);
            if (status != contact::FixedTriangleDiscoveryStatus::Ok)
                return {InitialFeatureSampleStatus::MaskFailure,
                        begin + local, SIZE_MAX,
                        contact::FixedTriangleArithmeticReason::None,
                        "Initial exact sample mask identity failed"};
        }
        next.task_mask_build_us += Microseconds(
            std::chrono::steady_clock::now() - mask_started);

        const auto discovery_started =
            std::chrono::steady_clock::now();
        const auto report = discovery.DiscoverMasked(
            triangles, triangle_count, sample_pairs + begin,
            count, task_masks);
        next.discovery_us += Microseconds(
            std::chrono::steady_clock::now() - discovery_started);
        if (report.status !=
                contact::FixedTriangleDiscoveryStatus::Ok) {
            const auto pair = report.input_pair == SIZE_MAX
                ? begin : begin + report.input_pair;
            return {InitialFeatureSampleStatus::DiscoveryFailure,
                    pair, report.input_task,
                    report.arithmetic_reason, report.message};
        }
        const auto features = discovery.features();
        const auto intersections = discovery.intersections();
        if (!features.complete || !intersections.complete ||
            features.count != report.feature_candidates ||
            intersections.count != report.intersections ||
            !Accumulate(report, next.complete) ||
            !Add(count, next.complete.sampled_pairs))
            return {InitialFeatureSampleStatus::Unrepresentable,
                    begin, SIZE_MAX,
                    contact::FixedTriangleArithmeticReason::None,
                    "Initial exact sample aggregate is unrepresentable"};

        Hash(begin, next.complete.feature_hash);
        Hash(features.count, next.complete.feature_hash);
        for (std::size_t feature = 0;
             feature < features.count; ++feature)
            Hash(features.data[feature],
                 next.complete.feature_hash);
        Hash(begin, next.complete.intersection_hash);
        Hash(intersections.count,
             next.complete.intersection_hash);
        for (std::size_t intersection = 0;
             intersection < intersections.count; ++intersection) {
            if (contact::RequiresIntersectionAdmission(
                    intersections.data[intersection])) {
                if (!next.complete.nonlocal_intersections) {
                    next.complete.first_nonlocal_intersection[0] =
                        intersections.data[intersection].triangles[0];
                    next.complete.first_nonlocal_intersection[1] =
                        intersections.data[intersection].triangles[1];
                }
                if (!Add(1, next.complete.nonlocal_intersections))
                    return {InitialFeatureSampleStatus::Unrepresentable,
                            begin + intersection, SIZE_MAX,
                            contact::FixedTriangleArithmeticReason::None,
                            "Initial nonlocal intersection count overflows"};
            }
            Hash(intersections.data[intersection],
                 next.complete.intersection_hash);
        }
        if (!begin) {
            next.worker_prefix = next.complete;
            next.worker_prefix_pairs = count;
        }
    }
    if (next.discovery_us)
        next.exact_tasks_per_second = static_cast<std::uint64_t>(
            (static_cast<long double>(
                 next.complete.exact_executed_tasks) * 1000000.0L) /
            next.discovery_us);
    *output = next;
    return {};
}

bool SameInitialFeatureSampleIdentity(
    const InitialFeatureSampleIdentity& first,
    const InitialFeatureSampleIdentity& second) noexcept {
    return first.sampled_pairs == second.sampled_pairs &&
        first.potential_tasks == second.potential_tasks &&
        first.local_masked_tasks == second.local_masked_tasks &&
        first.exact_executed_tasks ==
            second.exact_executed_tasks &&
        first.raw_feature_candidates ==
            second.raw_feature_candidates &&
        first.feature_candidates == second.feature_candidates &&
        first.raw_intersections == second.raw_intersections &&
        first.intersections == second.intersections &&
        first.nonlocal_intersections ==
            second.nonlocal_intersections &&
        Same(first.first_nonlocal_intersection[0],
             second.first_nonlocal_intersection[0]) &&
        Same(first.first_nonlocal_intersection[1],
             second.first_nonlocal_intersection[1]) &&
        first.feature_hash == second.feature_hash &&
        first.intersection_hash == second.intersection_hash;
}

}  // namespace crash::cases::vehicle_self_contact
