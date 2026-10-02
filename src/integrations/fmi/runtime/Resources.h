#pragma once

#include <string>

namespace robodyna::fmi {
// Resolve a declared FMU runfile. Simulation clocks and model parameters remain
// in the original driver; this adapter supplies only its packaged input path.
std::string ArchiveRunfile(const std::string& name);
}  // namespace robodyna::fmi
