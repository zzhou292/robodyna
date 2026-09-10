#pragma once
#include "ArtifactIO.h"
#include <array>

namespace crash::output {
struct SourcePartWallComparison {
    Document report;
    // 0: all gates and completed rebound; 2: qualified partial impact;
    // 1: a numerical/deformation/event gate failed. Invalid input throws.
    int exit_code=1;
};
SourcePartWallComparison CompareSourcePartWall(const std::array<std::filesystem::path,3>& directories);
} // namespace crash::output
