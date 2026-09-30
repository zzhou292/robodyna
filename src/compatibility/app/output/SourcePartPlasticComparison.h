#pragma once
#include "ArtifactIO.h"
#include <array>

namespace crash::output {
// Two validated source-plastic wall runs with the same physical configuration
// and a factor-two timestep change. Returns observations, never a convergence
// or numerical-promotion verdict. Invalid input throws before a report is returned.
Document CompareSourcePartPlastic(const std::array<std::filesystem::path,2>& directories);
}
