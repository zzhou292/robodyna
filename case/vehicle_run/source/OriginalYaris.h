#pragma once
#include "OriginalPaths.h"
#include "../Config.h"
#include "../ContactProfile.h"
#include "case/vehicle_wall/VehicleWallSetup.h"
#include "case/vehicle_startup/joints/VehicleJointModel.h"
#include <memory>
namespace crash::cases::vehicle_self_contact { class VehicleSelfContactSetup; }
namespace crash::cases::vehicle_run {
struct OriginalCase {
    vehicle_wall::VehicleWallSetup setup;
    vehicle_startup::joints::VehicleJointModel joints;
    std::shared_ptr<const vehicle_self_contact::VehicleSelfContactSetup> self_contact;
};
// Named pinned original profile. Composes existing typed source readers and
// factories; does not parse cards, invent model rows or allocate a solver owner.
// Search assessment uses the existing bounded CUDA startup broadphase.
OriginalCase PrepareOriginalYaris(const OriginalPaths&,vehicle_wall::Settings,
    PhysicalProfile=PhysicalProfile::RetainedShellAssembliesV1,
    ContactProfile=ContactProfile::WallOnly);
} // namespace crash::cases::vehicle_run
