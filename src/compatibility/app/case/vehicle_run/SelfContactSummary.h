#pragma once

#include "SelfContactTotals.h"
#include <iosfwd>
#include "case/vehicle_dynamics/VehiclePhysicalDynamics.h"

namespace crash::cases::vehicle_run {

// The caller first authenticates the actual committed interval using the output
// factory. This value check binds accepted-base force and candidate-policy
// diagnostics to that interval; it grants no publication authority. Failure
// preserves the prior totals, including when a late scalar is invalid.
void ObserveAcceptedSelfContact(SelfContactTotals&,
    const vehicle_dynamics::StepObservation&, const tl::fea::NodalStamp& accepted);

namespace detail {
// Fixed seven scalar fields, emitted only when an interval has been published.
void WriteSelfContactWorkProgress(std::ostream&, const SelfContactTotals&);
}

}  // namespace crash::cases::vehicle_run
