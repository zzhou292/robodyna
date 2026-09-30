#pragma once
#include "output/physical_run/WallComposition.h"
namespace crash::cases::vehicle_startup::physical_model {class VehiclePhysicalModel;}
namespace crash::cases::vehicle_wall {
// Read only the actual retained source/model and its immutable coefficient
// ledger. No caller-supplied profile strings or dynamic owner totals enter.
output::physical_run::WallComposition CaptureComposition(
    const vehicle_startup::physical_model::VehiclePhysicalModel&);
} // namespace crash::cases::vehicle_wall
