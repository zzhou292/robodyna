#pragma once
#include "OriginalPaths.h"
#include "case/vehicle_wall/VehicleWallSetup.h"
#include "case/vehicle_startup/joints/VehicleJointModel.h"
namespace crash::cases::vehicle_run {
struct OriginalCase {
    vehicle_wall::VehicleWallSetup setup;
    vehicle_startup::joints::VehicleJointModel joints;
};
// Named pinned original profile. Composes existing typed source readers and
// factories; does not parse cards, invent model rows or allocate a solver owner.
// Search assessment uses the existing bounded CUDA startup broadphase.
OriginalCase PrepareOriginalYaris(const OriginalPaths&,vehicle_wall::Settings);
} // namespace crash::cases::vehicle_run
