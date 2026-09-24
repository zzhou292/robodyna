#pragma once
#include "benchmarks/stage_timing/StageTimer.h"
#include <array>
#include <cstddef>
#include <cstdint>

namespace crash::cases::vehicle_run::contact_diagnostics {
// App-owned values keep the host-only controller/formatter independent of TL
// includes. These are executed-attempt observations, never physical authority.
inline constexpr std::size_t StageCount=9;
inline constexpr const char* StageNames[]{"setup","filtering","discovery","event_assembly",
    "force_assembly","residual","native_crossing","policy","finalization"};
static_assert(sizeof(StageNames)/sizeof(*StageNames)==StageCount);
inline constexpr std::size_t DiscoveryStageCount=7;
inline constexpr const char* DiscoveryStageNames[]{"input_ledger","input_sort","task_preparation",
    "geometry","result_fold","output_sort","publication"};
static_assert(sizeof(DiscoveryStageNames)/sizeof(*DiscoveryStageNames)==DiscoveryStageCount);
struct DiscoveryTiming {
    std::uint64_t calls=0,clock_failures=0,backward_samples=0;
    bool counter_saturated=false;
    std::array<benchmarks::StageCounter,DiscoveryStageCount> stages{};
};
struct Discovery {
    std::uint64_t calls=0;
    std::uint64_t failures=0;
    std::uint64_t triangle_references=0;
    std::uint64_t triangles=0;
    std::uint64_t vertex_references=0;
    std::uint64_t vertices=0;
    std::uint64_t edge_references=0;
    std::uint64_t edges=0;
    std::uint64_t raw_feature_candidates=0;
    std::uint64_t feature_candidates=0;
    std::uint64_t raw_intersections=0;
    std::uint64_t intersections=0;
    std::uint64_t potential_tasks=0;
    std::uint64_t local_masked_tasks=0;
    std::uint64_t exact_executed_tasks=0;
    DiscoveryTiming timing;
};
struct Phase {
    bool enabled=false,entered=false,finished=false,succeeded=false,authenticated=false;
    std::uint64_t owner_id=0,base_epoch=0,attempt=0;
    bool counter_saturated=false,counts_complete=true;
    std::uint64_t clock_failures=0,backward_samples=0;
    std::array<benchmarks::StageCounter,StageCount> stages{};
    Discovery discovery;
    std::uint64_t native_batches=0,native_submitted_pairs=0,native_work=0;
};
struct Snapshot {
    bool enabled=false,committed_scope=false,phase_matches=false;
    Phase accepted,candidate;
};
static_assert(sizeof(Snapshot)<=2048);
} // namespace crash::cases::vehicle_run::contact_diagnostics
