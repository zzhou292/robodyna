#include "AcceptedWall.h"
#include "case/vehicle_dynamics/VehiclePhysicalDynamics.h"
#include "case/vehicle_wall/VehicleWallSetup.h"
namespace crash::output::physical_run::detail {
cases::vehicle_wall::SetupIdentity CaptureWall(const cases::vehicle_dynamics::VehiclePhysicalDynamics& run,
        const records::Identity& id,const Values& v) {
    const auto* setup=run.wall_setup();
    const auto& step=run.last_accepted_step();
    const auto& w=step.wall;
    Require(bool(setup)==w.enabled,"Committed wall setup/observation presence differs");
    if(!setup) return {};
    CheckWallObservation(w,step.base,run.accepted(),id,v);
    return setup->identity();
}
} // namespace crash::output::physical_run::detail
