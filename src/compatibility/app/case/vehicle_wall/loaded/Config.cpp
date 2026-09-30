#include "../LoadedWall.h"
namespace crash::cases::vehicle_wall {
Settings LoadedWallSettings() noexcept {
    Settings settings;
    settings.mesh_profile=WallMeshProfile::EnvelopeRectangleV1;
    settings.transverse_margin_m=.25;
    return settings;
}
vehicle_dynamics::Config LoadedWallConfig() noexcept {
    vehicle_dynamics::Config config;
    config.startup.reserved_step_s=3e-7;
    config.structural={tl::fea::NodalCinStructuralProfile::NativeOrdinaryRigidTrace,.8};
    return config;
}
} // namespace crash::cases::vehicle_wall
