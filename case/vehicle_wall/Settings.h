#pragma once
#include "case/vehicle_runtime/Config.h"
#include "lib_src/collision/PlanarWallBox.h"
#include "lib_src/collision/NodalWallContactDevice.h"
#include <array>

namespace crash::cases::vehicle_wall {
enum class WallMeshProfile { PlacedOriginal, EnvelopeRectangleV1 };
struct Settings {
    WallMeshProfile mesh_profile=WallMeshProfile::EnvelopeRectangleV1;
    double initial_speed_mps=vehicle_runtime::InitialSpeedMps;
    double requested_duration_s=.02;
    double leading_gap_m=.02,transverse_margin_m=.01,exposed_clearance_m=1e-6;
    // Explicit numerical declaration; neither source mass nor a stability claim.
    double stiffness_per_area=1e7,maximum_penetration_m=.1;
    double parent_force_error=1e-6,parent_energy_error=1e-6;
    std::uint64_t wall_binding_id=0x594152495357414cULL;
};
struct PlacementValues {
    double translation_x_m=0,represented_wall_x_m=0;
    tlfea::contact::Q4IntegralInterval leading_gap;
    tlfea::contact::Q4IntegralInterval nominal_forward_travel;
    tlfea::contact::PlanarWallBox declared_world_envelope,projected_wall_box;
};
// Source-independent value checks. The world envelope is a declared runtime
// domain, not a prediction; complete finite-wall coverage is checked separately.
void CheckSettings(const Settings&);
PlacementValues Place(const std::array<tlfea::contact::Vec3,2>&,const Settings&);
tlfea::contact::NodalWallDeviceConfig ContactConfig(const Settings&,
    const tl::fea::NodalStamp&,std::uint64_t configuration,std::uint64_t qualification,
    const PlacementValues&);
} // namespace crash::cases::vehicle_wall
