#pragma once
#include "Progress.h"
#include "case/vehicle_dynamics/VehiclePhysicalDynamics.h"
namespace crash::cases::vehicle_run {
// Caller first authenticates the actual accepted interval through the existing
// output factory. These are peaks of reported endpoint observations, not an
// estimate of an unobserved continuous-time peak or total physical energy.
void ObserveAcceptedContact(ContactTotals&,const vehicle_dynamics::StepObservation&,
    const tl::fea::NodalStamp& accepted);
} // namespace crash::cases::vehicle_run
