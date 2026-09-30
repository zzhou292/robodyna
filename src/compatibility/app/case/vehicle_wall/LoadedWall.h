#pragma once
#include "VehicleWallStartup.h"
#include "WallStageError.h"
namespace crash::cases::vehicle_wall {
Settings LoadedWallSettings() noexcept;
vehicle_dynamics::Config LoadedWallConfig() noexcept;
// Creates/configures the existing dynamics class and installs one private
// finite-wall stage. All Prepare/Commit/Discard operations remain on that same
// returned owner. The factory is not a separate driver or physical clock.
class LoadedWall {
  public:
    static RuntimeForecast Preflight(const VehicleWallSetup&,
        vehicle_dynamics::Config=LoadedWallConfig(),RuntimeLimits={},
        const vehicle_runtime::JointModel* = nullptr);
    static vehicle_dynamics::VehiclePhysicalDynamics Prepare(const VehicleWallSetup&,
        vehicle_dynamics::Config=LoadedWallConfig(),RuntimeLimits={},
        const vehicle_runtime::JointModel* = nullptr);
  private:
    friend class crash::cases::vehicle_self_contact::LoadedWallSelfContact;
    struct Stages;
};
} // namespace crash::cases::vehicle_wall
