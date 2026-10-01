// Robodyna public API. These aliases preserve the inherited implementation.
#ifndef ROBODYNA_CORE_RBTYPES_H
#define ROBODYNA_CORE_RBTYPES_H

#include "chrono/core/ChTypes.h"

namespace robodyna::core {
// Preserve the inherited allocation/alignment policy.
using ::chrono_types::make_shared;
using ::chrono_types::make_unique;
}  // namespace robodyna::core

#endif
