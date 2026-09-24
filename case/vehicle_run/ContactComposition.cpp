#include "ContactComposition.h"
#include "Config.h"

#include "lib_src/elements/ShellBatchBinding.h"
#include "output/ArtifactIO.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace crash::cases::vehicle_run {
namespace {

namespace self = vehicle_self_contact;
namespace contact = tlfea::contact;

// Resource reservations for this bounded profile, not a predicted event census.
// Geometry is always counted completely and exhaustion remains a typed failure.
// In particular, no capacity is inferred from the first accepted event count.
constexpr std::size_t EventCapacity = 65536;
constexpr std::size_t ParentPairCapacity = 2000000;
constexpr std::size_t FacetPairCapacity = 8000000;
constexpr std::size_t FacetChunk = 4096;
constexpr std::size_t WorkPerPair = 4095;
constexpr std::size_t WorkPerChunk = std::size_t{1} << 20;
constexpr unsigned WorkerCount = 4;
constexpr std::uint64_t SelfSourceId = 0x524453454c465631ull;  // RDSELFV1

static_assert(FacetPairCapacity <= SIZE_MAX / WorkPerPair);
static_assert(EventCapacity <= SIZE_MAX / 2);
static_assert(sizeof(ContactComposition) <= 4096);

self::WallSelfContactLimits RuntimeLimits(
    const self::VehicleSelfContactSetup& setup) {
    self::WallSelfContactLimits result;
    const auto* shells = setup.physical().shells();
    output::Require(shells != nullptr,
        "Self-contact composition requires the retained shell binding");
    const contact::SelfContactTransactionLimits::ExactCensus census{
        setup.physical().domain()->node_count(),
        setup.surface().parents().size(),
        setup.active_uses().parents().size(),
        std::max({shells->qeph_count(), shells->t3_count(),
                  shells->qbat_count()}),
        setup.active_uses().facet_uses().size(),
        ParentPairCapacity, FacetPairCapacity, 0};
    auto& limits = result.self_contact.transaction;
    limits = contact::SelfContactTransactionLimits::Vehicle(
        census, FacetChunk, EventCapacity, 2 * EventCapacity, 0,
        WorkPerPair, WorkPerChunk, FacetPairCapacity * WorkPerPair, 20,
        result.host_bytes, result.device_bytes, result.host_bytes,
        WorkerCount, WorkerCount, EventCapacity);
    // Preserve the existing component profiles. The factory composes the
    // complete cost and the workstation guard independently admits GPU growth.
    limits.activity.max_host_bytes = std::size_t{512} << 20;
    limits.activity.max_startup_host_bytes = std::size_t{512} << 20;
    limits.broadphase.max_host_bytes = std::size_t{2} << 30;
    limits.broadphase.max_device_bytes = std::size_t{2} << 30;
    limits.regularity.max_host_bytes =
        contact::SelfContactCurrentRegularityLimits::Vehicle().max_host_bytes;
    output::Require(limits.max_host_bytes && limits.max_device_bytes,
        "Self-contact composition has an invalid source or capacity shape");
    return result;
}

void CheckSelfStep(const vehicle_dynamics::Config& dynamics) {
    output::Require(std::isfinite(dynamics.startup.reserved_step_s) && dynamics.startup.reserved_step_s>0,
        "Wall+self contact requires a finite positive physical step");
}

}  // namespace

ContactComposition ContactComposition::Prepare(
    ContactProfile profile,
    std::shared_ptr<const self::VehicleSelfContactSetup> source, bool enable_diagnostics,
    bool enable_cuda_facet_filters, bool enable_cuda_native_crossing) {
    ContactProfileName(profile);
    ContactComposition result;
    result.profile_ = profile;
    if (profile == ContactProfile::WallOnly) {
        output::Require(!enable_cuda_native_crossing, "CUDA native crossing cannot attach to wall-only composition");
        output::Require(!enable_cuda_facet_filters, "CUDA facet filters cannot attach to wall-only composition");
        output::Require(!enable_diagnostics, "Self-contact diagnostics cannot attach to wall-only composition");
        output::Require(!source,
            "Wall-only composition cannot hide a retained self-contact source");
        return result;
    }
    output::Require(bool(source) && source->config().facet_level == 0,
        "Wall+self-contact requires the authenticated level-0 source setup");
    result.runtime_config_ = {SelfSourceId, EventCapacity, 0};
    result.runtime_config_.enable_diagnostics = enable_diagnostics;
    result.runtime_config_.enable_cuda_facet_filters = enable_cuda_facet_filters;
    result.runtime_config_.enable_cuda_native_crossing = enable_cuda_native_crossing;
    if (enable_cuda_native_crossing) {
        result.runtime_config_.native_crossing_device_workers = NativeCrossingDeviceWorkers;
        result.runtime_config_.native_crossing_numeric_cohort_pairs = NativeCrossingNumericCohortPairs;
    }
    result.runtime_limits_ = RuntimeLimits(*source);
    result.self_contact_ = std::move(source);
    return result;
}

ContactCompositionForecast ContactComposition::Preflight(
    const vehicle_wall::VehicleWallSetup& wall,
    vehicle_dynamics::Config dynamics,
    const vehicle_runtime::JointModel* joints) const {
    ContactCompositionForecast result;
    if (profile_ == ContactProfile::WallOnly) {
        result.wall = vehicle_wall::LoadedWall::Preflight(
            wall, dynamics, {}, joints);
        result.retained_host_upper_bound = result.wall.retained_host_upper_bound;
        result.peak_host_upper_bound = result.wall.peak_host_upper_bound;
        result.device_bytes = result.wall.device_bytes;
        return result;
    }
    output::Require(bool(self_contact_),
        "Wall+self-contact composition lost its retained source");
    CheckSelfStep(dynamics);
    const auto combined = self::LoadedWallSelfContact::Preflight(
        wall, *self_contact_, runtime_config_, runtime_limits_, dynamics, joints);
    result.wall = combined.wall;
    result.self_contact = combined.self_contact;
    result.retained_host_upper_bound = combined.retained_host_upper_bound;
    result.peak_host_upper_bound = combined.peak_host_upper_bound;
    result.device_bytes = combined.device_bytes;
    return result;
}

vehicle_dynamics::VehiclePhysicalDynamics ContactComposition::CreateDynamics(
    const vehicle_wall::VehicleWallSetup& wall,
    vehicle_dynamics::Config dynamics,
    const vehicle_runtime::JointModel* joints) const {
    if (profile_ == ContactProfile::WallOnly)
        return vehicle_wall::LoadedWall::Prepare(wall, dynamics, {}, joints);
    output::Require(bool(self_contact_),
        "Wall+self-contact composition lost its retained source");
    CheckSelfStep(dynamics);
    return self::LoadedWallSelfContact::Prepare(
        wall, *self_contact_, runtime_config_, runtime_limits_, dynamics, joints);
}

}  // namespace crash::cases::vehicle_run
