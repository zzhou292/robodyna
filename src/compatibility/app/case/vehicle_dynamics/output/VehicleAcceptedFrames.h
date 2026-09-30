#pragma once
#include "../VehiclePhysicalDynamics.h"
#include "output/physical_frames/PhysicalAcceptedFrames.h"

namespace crash::cases::vehicle_dynamics::capture {
// Read-only presentation bridge to the dynamics-owned startup. The output
// producer retains its existing accepted-only phase/source checks; no owner
// reference, force operation or clock is exposed to the application caller.
// Serialize capture with all dynamics operations, including Prepare/Commit.
class VehicleAcceptedFrames {
  public:
    using Producer=crash::output::physical_frames::PhysicalAcceptedFrames;
    using Mapping=crash::output::physical_frames::Mapping;
    using Identity=crash::output::full_shell::Identity;
    using Limits=crash::output::physical_frames::Limits;
    VehicleAcceptedFrames(const Mapping&,VehiclePhysicalDynamics&,Identity,Limits={});
    void Capture(VehiclePhysicalDynamics&);
    const Producer& frames() const noexcept { return frames_; }
  private:
    static vehicle_runtime::VehiclePhysicalStartup& Startup(VehiclePhysicalDynamics&);
    Producer frames_;
};
} // namespace crash::cases::vehicle_dynamics::capture
