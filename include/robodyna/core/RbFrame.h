// Robodyna public API. These aliases preserve the inherited implementation.
#ifndef ROBODYNA_CORE_RBFRAME_H
#define ROBODYNA_CORE_RBFRAME_H

#include "chrono/core/ChFrame.h"

namespace robodyna::core {
template <class Real = double>
using RbFrame = ::chrono::ChFrame<Real>;
using RbFramed = ::chrono::ChFramed;
using RbFramef = ::chrono::ChFramef;
}  // namespace robodyna::core

#endif
