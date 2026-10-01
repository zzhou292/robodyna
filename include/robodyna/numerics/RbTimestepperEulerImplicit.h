// Robodyna public API. These aliases preserve the inherited implementation.
#ifndef ROBODYNA_NUMERICS_RBTIMESTEPPEREULERIMPLICIT_H
#define ROBODYNA_NUMERICS_RBTIMESTEPPEREULERIMPLICIT_H

#include "chrono/timestepper/ChTimestepperImplicit.h"

namespace robodyna::numerics {
using RbTimestepperEulerImplicit = ::chrono::ChTimestepperEulerImplicit;
using RbTimestepperEulerImplicitLinearized = ::chrono::ChTimestepperEulerImplicitLinearized;
using RbTimestepperEulerImplicitProjected = ::chrono::ChTimestepperEulerImplicitProjected;
}  // namespace robodyna::numerics

#endif
