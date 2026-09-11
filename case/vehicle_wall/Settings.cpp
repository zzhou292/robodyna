#include "Settings.h"
#include "case/wall_penalty/WallPlacementBounds.h"
#include "output/ArtifactIO.h"
#include <cmath>

namespace crash::cases::vehicle_wall {
namespace c = tlfea::contact;
void CheckSettings(const Settings& settings) {
    output::Require(settings.mesh_profile == WallMeshProfile::PlacedOriginal ||
        settings.mesh_profile == WallMeshProfile::EnvelopeRectangleV1,"Unknown explicit wall mesh profile");
    output::Require(settings.initial_speed_mps == vehicle_runtime::InitialSpeedMps &&
        std::isfinite(settings.requested_duration_s) && settings.requested_duration_s > 0 &&
        settings.requested_duration_s <= .05 &&
        (settings.requested_duration_s == .005 || settings.requested_duration_s == .02 ||
         settings.requested_duration_s == .05) && settings.wall_binding_id,
        "Vehicle wall profile requires 35 mph and an explicit 5, 20 or 50 ms duration");
    for (double value : {settings.leading_gap_m,settings.transverse_margin_m,settings.exposed_clearance_m,
                         settings.stiffness_per_area,settings.maximum_penetration_m,
                         settings.parent_force_error,settings.parent_energy_error}) {
        output::Require(std::isfinite(value) && value > 0,"Wall settings must be finite and positive");
    }
    output::Require(settings.exposed_clearance_m < settings.transverse_margin_m,
        "Wall clearance exceeds declared transverse margin");
}
PlacementValues Place(const std::array<c::Vec3,2>& bounds, const Settings& settings) {
    CheckSettings(settings);
    PlacementValues result;
    output::Require(wall_penalty::ExpandProjectedMotion(bounds,settings.transverse_margin_m,
        &result.declared_world_envelope),"Vehicle surface bounds or transverse envelope are invalid");
    const double desired = bounds[1].x + settings.leading_gap_m;
    result.translation_x_m = desired - .05;
    result.represented_wall_x_m = .05 + result.translation_x_m;
    output::Require(std::isfinite(desired) && std::isfinite(result.translation_x_m) &&
        wall_penalty::EncloseLeadingGap(result.represented_wall_x_m,bounds[1].x,&result.leading_gap),
        "Represented original wall translation cannot enclose a positive leading gap");
    output::Require(c::q4_bounds::Scale({settings.initial_speed_mps,settings.initial_speed_mps},
        settings.requested_duration_s,&result.nominal_forward_travel),"Declared nominal travel is unrepresentable");
    auto& world = result.declared_world_envelope;
    output::Require(c::q4_bounds::AddScalar(world.minimum.x,-settings.transverse_margin_m,false,&world.minimum.x) &&
        c::q4_bounds::AddScalar(world.maximum.x,result.nominal_forward_travel.upper,true,&world.maximum.x) &&
        c::q4_bounds::AddScalar(world.maximum.x,settings.transverse_margin_m,true,&world.maximum.x),
        "Declared world motion box overflows");
    result.projected_wall_box = world;
    result.projected_wall_box.minimum.x = result.represented_wall_x_m;
    result.projected_wall_box.maximum.x = result.represented_wall_x_m;
    return result;
}
c::NodalWallDeviceConfig ContactConfig(const Settings& settings, const tl::fea::NodalStamp& stamp,
    std::uint64_t configuration, std::uint64_t qualification, const PlacementValues& placement) {
    CheckSettings(settings);
    output::Require(configuration && qualification,"Wall runtime configuration identity is missing");
    c::NodalWallDeviceConfig result;
    result.owner = stamp;
    result.configuration_id = configuration;
    result.qualification_id = qualification;
    result.wall_binding_id = settings.wall_binding_id;
    result.law = {placement.represented_wall_x_m,settings.stiffness_per_area,settings.maximum_penetration_m,
                  settings.parent_force_error,settings.parent_energy_error};
    result.exposed_clearance = settings.exposed_clearance_m;
    result.limits = c::NodalWallDeviceLimits::Vehicle();
    result.max_device_bytes = c::MaxVehicleNodalWallDeviceBytes;
    result.max_host_bytes = c::MaxVehicleNodalWallHostBytes;
    return result;
}
} // namespace crash::cases::vehicle_wall
