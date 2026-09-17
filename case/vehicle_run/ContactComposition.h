#pragma once

#include "ContactProfile.h"
#include "case/vehicle_self_contact/SelfContactFactories.h"

#include <memory>
#include <optional>

namespace crash::cases::vehicle_run {

struct ContactCompositionForecast {
    vehicle_wall::RuntimeForecast wall;
    std::optional<vehicle_self_contact::RuntimeForecast> self_contact;
    std::size_t retained_host_upper_bound = 0;
    std::size_t peak_host_upper_bound = 0;
    std::size_t device_bytes = 0;
};

// Immutable selection of existing factories. The controller owns this small
// value in its fixed reserve; retained setup backing is charged by the factory.
// There is no mechanics owner, stepping clock or force implementation here.
class ContactComposition {
  public:
    ContactComposition() = default;
    static ContactComposition Prepare(
        ContactProfile,
        std::shared_ptr<const vehicle_self_contact::VehicleSelfContactSetup> = {});

    ContactCompositionForecast Preflight(
        const vehicle_wall::VehicleWallSetup&,
        vehicle_dynamics::Config,
        const vehicle_runtime::JointModel* = nullptr) const;
    vehicle_dynamics::VehiclePhysicalDynamics CreateDynamics(
        const vehicle_wall::VehicleWallSetup&,
        vehicle_dynamics::Config,
        const vehicle_runtime::JointModel* = nullptr) const;

    ContactProfile profile() const noexcept { return profile_; }
    const vehicle_self_contact::VehicleSelfContactSetup*
    self_contact_setup() const noexcept { return self_contact_.get(); }
    const vehicle_self_contact::RuntimeConfig&
    runtime_config() const noexcept { return runtime_config_; }
    const vehicle_self_contact::WallSelfContactLimits&
    runtime_limits() const noexcept { return runtime_limits_; }

  private:
    ContactProfile profile_ = ContactProfile::WallOnly;
    std::shared_ptr<const vehicle_self_contact::VehicleSelfContactSetup>
        self_contact_;
    vehicle_self_contact::RuntimeConfig runtime_config_;
    vehicle_self_contact::WallSelfContactLimits runtime_limits_;
};

}  // namespace crash::cases::vehicle_run
