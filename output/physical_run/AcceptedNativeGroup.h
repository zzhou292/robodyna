#pragma once
#include "Types.h"
namespace crash::cases::vehicle_dynamics { class VehiclePhysicalDynamics; }
namespace crash::output::physical_run::detail {
// Actual common-owner publication checks; no arbitrary values can construct an
// AcceptedInterval or stand in for the live native group.
void CaptureNativeGroup(const cases::vehicle_dynamics::VehiclePhysicalDynamics&,Profile,Values&);
}
