// Robodyna public API. These aliases preserve the inherited implementation.
#ifndef ROBODYNA_CORE_RBMATRIX33_H
#define ROBODYNA_CORE_RBMATRIX33_H

#include "chrono/core/ChMatrix33.h"

namespace robodyna::core {
template <class Real = double>
using RbMatrix33 = ::chrono::ChMatrix33<Real>;
using RbMatrix33d = ::chrono::ChMatrix33d;
using RbMatrix33f = ::chrono::ChMatrix33f;
}  // namespace robodyna::core

#endif
