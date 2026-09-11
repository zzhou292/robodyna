#pragma once
#include "../vehicle_runtime/Storage.h"
namespace crash::cases::vehicle_dynamics {
// Private case implementation seam; public runtime APIs expose no mutable owner.
struct ExecutionAccess {
    using State=vehicle_runtime::VehiclePhysicalStartup::Storage;
    static State& Get(vehicle_runtime::VehiclePhysicalStartup& value) noexcept { return *value.storage_; }
};
} // namespace crash::cases::vehicle_dynamics
