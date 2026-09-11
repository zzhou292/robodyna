#pragma once
#include "../vehicle_runtime/VehiclePhysicalStartup.h"
#include "MotionSummary.h"
#include <stdexcept>

namespace crash::cases::vehicle_wall { class VehicleWallStartup; }
namespace crash::cases::vehicle_dynamics {
namespace capture { class VehicleAcceptedFrames; }
struct Config {
    vehicle_runtime::Config startup;
    std::size_t workspace_bytes=128u<<20;
    double maximum_rotation_increment=.2;
    tl::fea::NodalCinStructuralStep structural;
};
struct Forecast {
    vehicle_runtime::Forecast startup;
    std::size_t workspace_bytes=0,peak_host_upper_bound=0;
};
struct StepObservation {
    tl::fea::NodalStamp base;
    double proposed_time=0;
    MotionSummary uniform_motion;
    tl::fea::ShellPhysicalDiagnostics mechanics;
    // Zero with the disabled profile; otherwise actual post-CIN local limit.
    double structural_step_limit=0;
};
class StepSizeError : public std::runtime_error {
  public:
    StepSizeError(double limit,std::uint32_t node):std::runtime_error("Physical timestep exceeds the post-CIN limit"),
        limit_(limit),node_(node) {}
    double limit() const noexcept {return limit_;}
    std::uint32_t node() const noexcept {return node_;}
  private:
    double limit_;
    std::uint32_t node_;
};
// Composes the currently selected physical operators through one TL clock.
// This initial integration profile has no external loads, joints or contact.
// It qualifies complete-model free flight; it does not admit a connected crash.
// PrepareStep does all fallible mechanics/readback work; CommitStep publishes
// the sole owner and all six histories. DiscardStep preserves accepted results.
class VehiclePhysicalDynamics {
  public:
    static Forecast Preflight(const vehicle_runtime::Execution&,const vehicle_runtime::Attachments&,Config={});
    static VehiclePhysicalDynamics Prepare(const vehicle_runtime::Execution&,const vehicle_runtime::Attachments&,Config={});
    ~VehiclePhysicalDynamics();
    VehiclePhysicalDynamics(VehiclePhysicalDynamics&&) noexcept;
    VehiclePhysicalDynamics& operator=(VehiclePhysicalDynamics&&) noexcept;
    VehiclePhysicalDynamics(const VehiclePhysicalDynamics&)=delete;
    VehiclePhysicalDynamics& operator=(const VehiclePhysicalDynamics&)=delete;
    const Forecast& forecast() const noexcept;
    tl::fea::NodalStamp accepted() const noexcept;
    tl::fea::NodalAllocationInfo allocations() const noexcept;
    const StepObservation& PrepareStep();
    void CommitStep();
    void DiscardStep() noexcept;
    bool has_prepared_step() const noexcept;
    const StepObservation& last_accepted_step() const;
  private:
    friend class capture::VehicleAcceptedFrames;
    friend class vehicle_wall::VehicleWallStartup;
    struct Storage;
    explicit VehiclePhysicalDynamics(std::unique_ptr<Storage>);
    std::unique_ptr<Storage> storage_;
};
} // namespace crash::cases::vehicle_dynamics
