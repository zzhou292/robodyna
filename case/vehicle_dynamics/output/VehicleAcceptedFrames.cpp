#include "VehicleAcceptedFrames.h"
#include "../Storage.h"

namespace crash::cases::vehicle_dynamics::capture {
vehicle_runtime::VehiclePhysicalStartup& VehicleAcceptedFrames::Startup(VehiclePhysicalDynamics& run) {
    crash::output::Require(bool(run.storage_),"Accepted capture requires a live dynamics owner");
    return run.storage_->startup;
}
VehicleAcceptedFrames::VehicleAcceptedFrames(const Mapping& map,VehiclePhysicalDynamics& run,
    Identity identity,Limits limits):frames_(map,Startup(run),std::move(identity),limits) {}
void VehicleAcceptedFrames::Capture(VehiclePhysicalDynamics& run) {
    frames_.Capture(Startup(run));
}
} // namespace crash::cases::vehicle_dynamics::capture
