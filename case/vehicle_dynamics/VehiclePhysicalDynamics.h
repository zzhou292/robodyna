#pragma once
#include "../vehicle_runtime/VehiclePhysicalStartup.h"
#include "MotionSummary.h"
#include "lib_src/solvers/NodalUniformMotionObserver.h"
#include "SelfContactObservation.h"
#include "WallObservation.h"
#include "StepTiming.h"
#include "native_contact/Observation.h"
#include "lib_src/solvers/NodalCinStructuralLimit.h"
#include <stdexcept>

namespace crash::cases::vehicle_wall { class VehicleWallStartup; class VehicleWallSetup; class LoadedWall; struct RuntimeForecast; }
namespace crash::cases::vehicle_self_contact {
class VehicleSelfContactStartup;
class VehicleSelfContactSetup;
class SelfContactOnly;
class LoadedWallSelfContact;
class VehicleSelfContactInitialCensus;
class CandidateRigidCouponAccess;
struct RuntimeForecast;
}
namespace crash::cases::vehicle_native_contact { class VehicleContactStartup; }
namespace crash::cases::vehicle_dynamics {
namespace native_contact { class Group; }
struct AllocationInfo {
    std::size_t device_bytes=0,device_allocations=0;
    // Byte accounting is complete. The native transaction reports payload
    // bytes but has no allocator-count API; its presence makes this false.
    bool device_allocation_count_complete=true;
};
namespace capture { class VehicleAcceptedFrames; }
struct Config {
    vehicle_runtime::Config startup;
    std::size_t workspace_bytes=128u<<20;
    double maximum_rotation_increment=.2;
    tl::fea::NodalCinStructuralStep structural;
    StepTimingOptions timing;
    tl::fea::NodalUniformMotionLimits motion_limits;
};
struct Forecast {
    vehicle_runtime::Forecast startup;
    tl::fea::NodalUniformMotionForecast motion;
    std::size_t workspace_bytes=0,peak_host_upper_bound=0;
};
struct StepObservation {
    tl::fea::NodalStamp base;
    double proposed_time=0;
    MotionSummary uniform_motion;
    tl::fea::ShellPhysicalDiagnostics mechanics;
    // Zero with the disabled profile; otherwise actual post-CIN local limit.
    double structural_step_limit=0;
    // Unavailable by default. Copied from the same successful prepared owner
    // attempt and published by the existing accepted observation swap.
    tl::fea::NodalCinStructuralLimit structural_limiter;
    WallObservation wall;
    SelfContactObservation self_contact;
    bool motion_includes_fixed_environment=false;
    native_contact::GroupObservation native_contact;
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
// The original factory remains free flight. The explicit loaded factory adds
// one finite-wall stage to this same transaction, with actual step screening.
// This class alone does not establish complete source/load-path closure.
// PrepareStep does all fallible mechanics/readback work; CommitStep publishes
// the sole owner and every declared history. DiscardStep preserves accepted results.
class VehiclePhysicalDynamics {
  public:
    static Forecast Preflight(const vehicle_runtime::Source&,Config={});
    static VehiclePhysicalDynamics Prepare(const vehicle_runtime::Source&,Config={});
    static Forecast Preflight(const vehicle_runtime::Execution&,const vehicle_runtime::Attachments&,
        Config={},const vehicle_runtime::JointModel* = nullptr);
    static VehiclePhysicalDynamics Prepare(const vehicle_runtime::Execution&,const vehicle_runtime::Attachments&,
        Config={},const vehicle_runtime::JointModel* = nullptr);
    ~VehiclePhysicalDynamics();
    VehiclePhysicalDynamics(VehiclePhysicalDynamics&&) noexcept;
    VehiclePhysicalDynamics& operator=(VehiclePhysicalDynamics&&) noexcept;
    VehiclePhysicalDynamics(const VehiclePhysicalDynamics&)=delete;
    VehiclePhysicalDynamics& operator=(const VehiclePhysicalDynamics&)=delete;
    const Forecast& forecast() const noexcept;
    tl::fea::NodalStamp accepted() const noexcept;
    AllocationInfo allocations() const noexcept;
    const native_contact::Group* native_contact_group() const noexcept;
    StepTimingSnapshot timing() const noexcept;
    // Null on the unchanged free-flight profile. Returned source/budget views
    // remain immutable and valid while the dynamics object is alive.
    const vehicle_wall::VehicleWallSetup* wall_setup() const noexcept;
    const vehicle_wall::RuntimeForecast* wall_forecast() const noexcept;
    const vehicle_self_contact::VehicleSelfContactSetup*
        self_contact_setup() const noexcept;
    const vehicle_self_contact::RuntimeForecast*
        self_contact_forecast() const noexcept;
    tlfea::contact::SelfContactTransactionAllocationInfo
        self_contact_allocations() const noexcept;
    tlfea::contact::SelfContactTransactionDiagnostics self_contact_diagnostics() const noexcept;
    const StepObservation& PrepareStep();
    void CommitStep();
    void DiscardStep() noexcept;
    bool has_prepared_step() const noexcept;
    const StepObservation& last_accepted_step() const;
  private:
    friend class capture::VehicleAcceptedFrames;
    friend class vehicle_native_contact::VehicleContactStartup;
    // Only the typed case factory may attach initialized native sources. This
    // reauthenticates the real owner and installs one exclusive native roster.
    void InstallNativeContact(std::unique_ptr<native_contact::Group>);
    friend class vehicle_wall::VehicleWallStartup;
    friend class vehicle_wall::LoadedWall;
    friend class vehicle_self_contact::VehicleSelfContactStartup;
    friend class vehicle_self_contact::SelfContactOnly;
    friend class vehicle_self_contact::LoadedWallSelfContact;
    friend class vehicle_self_contact::VehicleSelfContactInitialCensus;
    friend class vehicle_self_contact::CandidateRigidCouponAccess;
    struct Storage;
    explicit VehiclePhysicalDynamics(std::unique_ptr<Storage>);
    std::unique_ptr<Storage> storage_;
};
} // namespace crash::cases::vehicle_dynamics
