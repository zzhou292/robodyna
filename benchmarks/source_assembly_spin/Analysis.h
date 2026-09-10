#pragma once
#include "output/ArtifactIO.h"
#include <filesystem>
namespace crash::benchmarks::assembly_spin {
// Bounded original six-part source + actual paired force-stage trace. This is
// diagnostic native replay, not a restart, new owner or guard authorization.
output::Document Analyze(const std::filesystem::path& inventory,const std::filesystem::path& trace);
}
