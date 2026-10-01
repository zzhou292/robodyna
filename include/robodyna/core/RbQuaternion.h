// Robodyna public API. These aliases preserve the inherited implementation.
#ifndef ROBODYNA_CORE_RBQUATERNION_H
#define ROBODYNA_CORE_RBQUATERNION_H

#include "chrono/core/ChQuaternion.h"

namespace robodyna::core {
template <class Real = double>
using RbQuaternion = ::chrono::ChQuaternion<Real>;
using RbQuaterniond = ::chrono::ChQuaterniond;
using RbQuaternionf = ::chrono::ChQuaternionf;
}  // namespace robodyna::core

#endif
