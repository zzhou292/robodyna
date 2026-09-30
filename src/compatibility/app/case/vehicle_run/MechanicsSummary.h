#pragma once
#include "MechanicsTotals.h"
#include "case/vehicle_dynamics/VehiclePhysicalDynamics.h"

namespace crash::cases::vehicle_run {
// Called after the existing CaptureAcceptedInterval source/owner authentication.
// Consumes only the committed StepObservation, without history readback or any
// new clock. Validation or arithmetic failure leaves the entire output intact.
void ObserveAcceptedMechanics(MechanicsTotals&, const vehicle_dynamics::StepObservation&,
    const tl::fea::NodalStamp& accepted);
} // namespace crash::cases::vehicle_run
