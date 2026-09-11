#pragma once
#include "VehicleWallSetup.h"
#include <filesystem>
namespace crash::cases::vehicle_wall {
// Caller-owned empty directory, create-only files. Writes the actual selected
// Chrono triangle mesh and its separate setup/source identity. No completed-run
// marker, runtime state or publication is produced. Failure can leave files.
void WriteSetupArtifacts(const std::filesystem::path&, const VehicleWallSetup&);
} // namespace crash::cases::vehicle_wall
