#pragma once
#include "Types.h"
#include "case/vehicle_wall/SetupIdentity.h"
namespace crash::cases::vehicle_dynamics {class VehiclePhysicalDynamics;}
namespace crash::output::physical_frames {class PhysicalAcceptedFrames;}
namespace crash::output::physical_run {
// Only the live accepted-state factory constructs this handle. File readers
// return Values for observation, not a token that can append another run.
class AcceptedInterval {
  public:
    const Values& values() const noexcept {return values_;}
    const records::Identity& identity() const noexcept {return identity_;}
    Profile profile() const noexcept {return profile_;}
  private:
    AcceptedInterval(Values v,records::Identity i,Profile p,cases::vehicle_wall::SetupIdentity wall)
        :values_(std::move(v)),identity_(std::move(i)),profile_(p),wall_(std::move(wall)) {}
    Values values_;
    records::Identity identity_;
    Profile profile_;
    cases::vehicle_wall::SetupIdentity wall_;
    friend class RunArchive;
    friend AcceptedInterval CaptureAcceptedInterval(const cases::vehicle_dynamics::VehiclePhysicalDynamics&,
        const physical_frames::PhysicalAcceptedFrames&,Profile);
};
AcceptedInterval CaptureAcceptedInterval(const cases::vehicle_dynamics::VehiclePhysicalDynamics&,
    const physical_frames::PhysicalAcceptedFrames&,Profile);
} // namespace crash::output::physical_run
