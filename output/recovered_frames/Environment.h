#pragma once
#include "Internal.h"
namespace crash::output::recovered_frames {
// Inspect only the three original environment records. Full source/mesh
// authentication is performed by the existing normal environment reader.
run::EnvironmentReceipt InspectEnvironment(const std::filesystem::path&,const run::Configuration&);
void CheckStaticProfiles(const run::Configuration&,const Description&);
} // namespace crash::output::recovered_frames
