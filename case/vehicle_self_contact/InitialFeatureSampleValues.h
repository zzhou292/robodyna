#pragma once

#include "lib_src/collision/FixedTriangleFeatureDiscovery.h"
#include <cstddef>
#include <cstdint>

namespace crash::cases::vehicle_self_contact {

inline constexpr std::size_t InitialExactFeatureSampleCapacity = 65536;
inline constexpr std::size_t InitialExactFeatureChunkCapacity = 4096;
inline constexpr std::size_t InitialExactFeatureHostByteCap =
    std::size_t{128} << 20;
inline constexpr std::uint64_t InitialExactFeatureRerunThresholdUs =
    std::uint64_t{120} * 1000 * 1000;

struct InitialFeatureSampleIdentity {
    std::size_t sampled_pairs = 0;
    std::size_t potential_tasks = 0;
    std::size_t local_masked_tasks = 0;
    std::size_t exact_executed_tasks = 0;
    std::size_t raw_feature_candidates = 0;
    std::size_t feature_candidates = 0;
    std::size_t raw_intersections = 0;
    std::size_t intersections = 0;
    std::uint64_t feature_hash = 0;
    std::uint64_t intersection_hash = 0;
};

struct InitialFeatureSampleResult {
    InitialFeatureSampleIdentity complete;
    InitialFeatureSampleIdentity worker_prefix;
    std::size_t worker_prefix_pairs = 0;
    std::uint64_t task_mask_build_us = 0;
    std::uint64_t discovery_us = 0;
    std::uint64_t exact_tasks_per_second = 0;
};

enum class InitialFeatureSampleStatus {
    Ok,
    InvalidInput,
    MaskFailure,
    DiscoveryFailure,
    Unrepresentable
};

struct InitialFeatureSampleReport {
    InitialFeatureSampleStatus status = InitialFeatureSampleStatus::Ok;
    std::size_t pair = SIZE_MAX;
    std::size_t task = SIZE_MAX;
    tlfea::contact::FixedTriangleArithmeticReason arithmetic_reason =
        tlfea::contact::FixedTriangleArithmeticReason::None;
    const char* message = "OK";
};

tlfea::contact::FixedTriangleFeatureLimits
InitialFeatureSampleDiscoveryLimits(
    std::size_t chunk_capacity,
    unsigned worker_count) noexcept;

InitialFeatureSampleReport DiscoverInitialFeatureSample(
    tlfea::contact::FixedTriangleFeatureDiscovery& discovery,
    const tlfea::contact::CurrentFixedTriangle* triangles,
    std::size_t triangle_count,
    const tlfea::contact::FixedTrianglePair* sample_pairs,
    std::size_t sample_pair_count,
    tlfea::contact::FixedTriangleFeatureTaskMask* task_masks,
    std::size_t chunk_capacity,
    InitialFeatureSampleResult* output) noexcept;

bool SameInitialFeatureSampleIdentity(
    const InitialFeatureSampleIdentity& first,
    const InitialFeatureSampleIdentity& second) noexcept;

}  // namespace crash::cases::vehicle_self_contact
