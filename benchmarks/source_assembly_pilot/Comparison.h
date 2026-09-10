#pragma once
#include "output/ArtifactIO.h"
#include <vector>

namespace crash::benchmarks::assembly_pilot {
// Reference first, then one or two independently validated candidate bundles.
// Reports differences over exact common saved times, including accepted
// prefixes. Throws on malformed/incompatible input; no convergence threshold.
output::Document Compare(const std::vector<std::filesystem::path>& directories);
} // namespace crash::benchmarks::assembly_pilot
