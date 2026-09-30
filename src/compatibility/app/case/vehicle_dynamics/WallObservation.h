#pragma once
#include "lib_src/collision/NodalWallMappedContact.h"
namespace crash::cases::vehicle_dynamics {
struct WallObservation {
    // False is unavailable/no contact participant, not a measured zero load.
    bool enabled=false;
    tlfea::contact::NodalWallMappedDiagnostics accepted;
    tlfea::contact::NodalWallMappedDiagnostics prepared;
};
} // namespace crash::cases::vehicle_dynamics
