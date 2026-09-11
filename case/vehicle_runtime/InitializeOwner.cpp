#include "Storage.h"
#include "Packing.h"
#include "Reports.h"
namespace crash::cases::vehicle_runtime {
void VehiclePhysicalStartup::Storage::InitializeOwner() {
    const auto packing = detail::PackOwner(execution.model().coefficients(),execution.model().rigid_assembly(),
        roles,{InitialSpeedMps,0,0},forecast.packing_bytes);
    const auto cin = detail::CinStartup(config,attachments,packing.mass.data(),packing.inertia.data());
    detail::RequireSuccess(owner.Initialize(detail::OwnerConfig(config,roles.node.size()),packing.kinematics(),
        packing.inverse_mass.data(),packing.dofs(),execution.model().rigid_assembly(),&cin));
    // Packing retires before any participant allocates its upload/proof buffers.
}
} // namespace crash::cases::vehicle_runtime
