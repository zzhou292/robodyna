#pragma once

#include "VehicleSelfContactStartup.h"
#include "case/vehicle_wall/LoadedWall.h"

namespace crash::cases::vehicle_self_contact {

// Public self-only runtime factory used by bounded qualification paths. It
// installs one private contribution into the existing dynamics owner.
class SelfContactOnly {
  public:
    static RuntimeForecast Preflight(
        const VehicleSelfContactSetup&, vehicle_dynamics::Config,
        RuntimeConfig, RuntimeLimits,
        const vehicle_runtime::JointModel* = nullptr);
    static vehicle_dynamics::VehiclePhysicalDynamics Prepare(
        const VehicleSelfContactSetup&, vehicle_dynamics::Config,
        RuntimeConfig, RuntimeLimits,
        const vehicle_runtime::JointModel* = nullptr);
};

struct WallSelfContactLimits {
    std::size_t host_bytes =
        std::size_t{20} * 1000 * 1000 * 1000;
    std::size_t device_bytes = std::size_t{8} << 30;
    vehicle_wall::RuntimeLimits wall;
    RuntimeLimits self_contact;
    tl::fea::ShellPhysicalScratchParticipationLimits participation;
};

struct WallSelfContactForecast {
    vehicle_wall::RuntimeForecast wall;
    RuntimeForecast self_contact;
    tl::fea::ShellPhysicalScratchParticipationForecast participation;
    std::size_t retained_host_upper_bound = 0;
    std::size_t peak_host_upper_bound = 0;
    std::size_t device_bytes = 0;
};

// Opt-in loaded Yaris composition. It is intentionally not wired to the
// long-run CLI until full-V5 overlap and exact-capacity gates are available.
class LoadedWallSelfContact {
  public:
    static WallSelfContactForecast Preflight(
        const vehicle_wall::VehicleWallSetup&,
        const VehicleSelfContactSetup&,
        RuntimeConfig, WallSelfContactLimits,
        vehicle_dynamics::Config =
            vehicle_wall::LoadedWallConfig(),
        const vehicle_runtime::JointModel* = nullptr);
    static vehicle_dynamics::VehiclePhysicalDynamics Prepare(
        const vehicle_wall::VehicleWallSetup&,
        const VehicleSelfContactSetup&,
        RuntimeConfig, WallSelfContactLimits,
        vehicle_dynamics::Config =
            vehicle_wall::LoadedWallConfig(),
        const vehicle_runtime::JointModel* = nullptr);
};

}  // namespace crash::cases::vehicle_self_contact
