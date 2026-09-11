#include "Stages.h"
#include "case/vehicle_dynamics/Storage.h"
#include "output/ArtifactIO.h"
namespace crash::cases::vehicle_wall {
RuntimeForecast LoadedWall::Preflight(const VehicleWallSetup& setup,
    vehicle_dynamics::Config config,RuntimeLimits limits,const vehicle_runtime::JointModel* joints) {
    output::Require(setup.settings().mesh_profile==WallMeshProfile::EnvelopeRectangleV1 &&
        setup.settings().transverse_margin_m>=.25,
        "Loaded wall requires the explicit complete mesh and at least 0.25 m transverse margin");
    output::Require(config.structural.profile==tl::fea::NodalCinStructuralProfile::NativeOrdinaryRigidTrace &&
        tl::fea::ValidCinStructuralStep(config.structural),
        "Loaded wall requires the explicit post-CIN current-coefficient step screen");
    auto result=VehicleWallStartup::Preview(setup,config,limits,joints);
    const auto extra=sizeof(Stages)+256;
    output::Require(extra<=limits.host_bytes && result.peak_host_upper_bound<=limits.host_bytes-extra,
        "Loaded wall stage storage exceeds the complete host cap");
    result.retained_host_upper_bound+=extra;
    result.peak_host_upper_bound+=extra;
    return result;
}
vehicle_dynamics::VehiclePhysicalDynamics LoadedWall::Prepare(const VehicleWallSetup& setup,
    vehicle_dynamics::Config config,RuntimeLimits limits,const vehicle_runtime::JointModel* joints) {
    const auto forecast=Preflight(setup,config,limits,joints);
    auto dynamics=vehicle_dynamics::VehiclePhysicalDynamics::Prepare(setup.execution(),setup.attachments(),config,joints);
    auto contact=VehicleWallStartup::Prepare(setup,dynamics,limits);
    dynamics.storage_->wall=std::make_unique<Stages>(std::move(contact),forecast);
    dynamics.storage_->forecast.peak_host_upper_bound=forecast.peak_host_upper_bound;
    return dynamics;
}
} // namespace crash::cases::vehicle_wall
