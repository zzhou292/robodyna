#pragma once
#include "Types.h"
namespace crash::cases::vehicle_dynamics {class VehiclePhysicalDynamics;}
namespace crash::output::physical_run::detail {
// Called only after the common committed owner/structural stamp was checked.
// Authenticates live participant identity before copying pointer-free values.
void CaptureSelfContact(const cases::vehicle_dynamics::VehiclePhysicalDynamics&,
    const records::Identity&,Profile,Values&);
}
