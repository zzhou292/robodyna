// Robodyna public API. These aliases preserve the inherited implementation.
#ifndef ROBODYNA_CORE_RBVECTOR3_H
#define ROBODYNA_CORE_RBVECTOR3_H

#include "chrono/core/ChVector3.h"

namespace robodyna::core {
template <class Real = double>
using RbVector3 = ::chrono::ChVector3<Real>;
using RbVector3d = ::chrono::ChVector3d;
using RbVector3f = ::chrono::ChVector3f;
using RbVector3i = ::chrono::ChVector3i;
using RbVector3b = ::chrono::ChVector3b;

using ::chrono::Vdot;
using ::chrono::Vcross;
}  // namespace robodyna::core

#endif
