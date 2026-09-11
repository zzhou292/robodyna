#pragma once
#include "ReplayGeometryLimits.h"
#include "chrono/core/ChVector3.h"
#include <vector>
namespace crash::visual {
// Shared exact Chrono/VSG representability check. This is display admission,
// independent of source authentication, solver geometry and accepted state.
bool CheckReplayDisplayPositions(const std::vector<chrono::ChVector3d>&, ReplayGeometryLimits = {});
bool CheckReplayDisplayGeometry(const std::vector<chrono::ChVector3d>&,
    const std::vector<chrono::ChVector3i>&, ReplayGeometryLimits = {});
} // namespace crash::visual
