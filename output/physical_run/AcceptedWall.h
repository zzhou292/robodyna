#pragma once
#include "Types.h"
#include "case/vehicle_wall/SetupIdentity.h"
namespace crash::cases::vehicle_dynamics {class VehiclePhysicalDynamics;}
namespace tl::fea {struct NodalStamp;}
namespace crash::cases::vehicle_dynamics {struct WallObservation;}
namespace crash::output::physical_run::detail {
void CheckWallObservation(const cases::vehicle_dynamics::WallObservation&,const tl::fea::NodalStamp& base,
    const tl::fea::NodalStamp& accepted,const records::Identity&,const Values&);
cases::vehicle_wall::SetupIdentity CaptureWall(const cases::vehicle_dynamics::VehiclePhysicalDynamics&,
    const records::Identity&,const Values&);
} // namespace crash::output::physical_run::detail
