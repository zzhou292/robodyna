#include "Fixture.h"
#include "lib_utest/qualification/nodal_rigid_group/PreparedSnapshotCudaProbe.h"
#include <limits>

namespace crash::cases::source_assembly_dynamics {
observation::ForceStageInput SourceAssemblyDynamicsTestAccess::CapturedInput(
        const SourceAssemblyWallCase& c,const Sample& before) {
    const auto& s=*c.impl_;observation::ForceStageInput input;
    input.bindings=&s.bindings;input.base=before.diagnostics.stamp;input.before_group_stamp=before.group_stamp;
    input.before=before.fields.view();input.before_groups=before.fields.groups.data();
    input.force_groups=s.accepted().fields.groups.data();input.group_count=s.groups();
    input.prepared=s.prepared;input.frame_prepared=s.group_prepared;input.capture_prepared=s.force_capture.prepared;
    input.acceleration_xyz=s.force_capture.acceleration.data();
    input.angular_acceleration_xyz=input.acceleration_xyz+3*s.nodes();
    input.group_acceleration=s.force_capture.groups.data();input.acceleration_nodes=s.nodes();input.acceleration_groups=s.groups();
    return input;
}
std::array<std::uintptr_t,4> SourceAssemblyDynamicsTestAccess::CaptureStorage(const SourceAssemblyWallCase& c) {
    const auto& w=c.impl_->force_capture;
    return {reinterpret_cast<std::uintptr_t>(w.acceleration.data()),reinterpret_cast<std::uintptr_t>(w.groups.data()),
            w.acceleration.capacity(),w.groups.capacity()};
}
Report SourceAssemblyDynamicsTestAccess::RejectForceStage(SourceAssemblyWallCase& c,ForceFault fault) {
    auto& s=*c.impl_;auto r=s.Prepare();if(!r)return s.Stop(r);
    r=s.Evaluate();if(!r)return s.Stop(r);
    if(fault==ForceFault::CaptureAssociation) {
        ++s.group_prepared.attempt;
        r=s.CaptureForceStage();
        if(r)return s.Stop(Failure(Status::ComponentFailure,"Injected capture association fault unexpectedly passed"));
        return s.Stop(r);
    }
    const double invalid=std::numeric_limits<double>::quiet_NaN();
    if(fault==ForceFault::LastNode)s.force_capture.acceleration.back()=invalid;
    if(fault==ForceFault::LastGroup)s.force_capture.groups.back().angular_acceleration.z=invalid;
    if(fault==ForceFault::CaptureIdentity)++s.force_capture.prepared.attempt;
    if(fault==ForceFault::GroupIdentity)++s.group_prepared.attempt;
    r=s.Check();
    if(r)return s.Stop(Failure(Status::ComponentFailure,"Injected force-stage input fault unexpectedly passed"));
    return s.Stop(r);
}
Report SourceAssemblyDynamicsTestAccess::RejectCaptureDevice(SourceAssemblyWallCase& c) {
    auto& s=*c.impl_;auto r=s.Prepare();if(!r)return s.Stop(r);
    r=s.Evaluate();if(!r)return s.Stop(r);
    // Ready-phase readback is supported by the owner. Fail this actual refresh
    // only after its real D2H transfer completes; no production fault hook.
    prepared_snapshot_probe::FailAfterNextDeviceRead();
    r=s.CaptureForceStage();
    if(r)return s.Stop(Failure(Status::ComponentFailure,"Injected CUDA capture fault unexpectedly passed"));
    return s.Stop(r);
}
} // namespace crash::cases::source_assembly_dynamics
