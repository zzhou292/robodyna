#pragma once
#include "ReplayGeometryLimits.h"
#include "robodyna/core/RbVector3.h"
#include <vector>
namespace crash::visual {
// Shared exact Chrono/VSG representability check. This is display admission,
// independent of source authentication, solver geometry and accepted state.
bool CheckReplayDisplayPositions(const std::vector<robodyna::core::RbVector3d>&, ReplayGeometryLimits = {});
bool CheckReplayDisplayGeometry(const std::vector<robodyna::core::RbVector3d>&,
    const std::vector<robodyna::core::RbVector3i>&, ReplayGeometryLimits = {});
} // namespace crash::visual
