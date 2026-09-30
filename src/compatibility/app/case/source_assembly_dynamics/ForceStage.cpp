#include "State.h"

namespace crash::cases::source_assembly_dynamics {
Report SourceAssemblyWallCase::Impl::CaptureForceStage() {
    if(!config.observe_force_stage)return Success();
    if(force_capture.acceleration.size()!=6*nodes()||force_capture.groups.size()!=groups())
        return Failure(Status::ResourceLimit,"Force-stage workspace lost its exact admitted extent");
    const auto r=Convert(owner.CopyPreparedForceStage(token,force_capture.buffer(),&force_capture.prepared));
    if(!r)return r;
    if(!fe::trial_identity::SamePrepared(prepared,group_prepared)||
       !fe::trial_identity::SamePrepared(prepared,force_capture.prepared))
        return Failure(Status::ComponentFailure,"Actual force-stage/node/group readbacks identify different prepared states");
    return Success();
}
Report SourceAssemblyWallCase::Impl::CheckForceStage() {
    if(!config.observe_force_stage)return Success();
    const auto& old=accepted();auto& next=candidate();
    observation::ForceStageInput input;
    input.bindings=&bindings;input.base=old.diagnostics.stamp;input.before_group_stamp=old.group_stamp;
    input.before=old.fields.view();input.before_groups=old.fields.groups.data();
    input.force_groups=next.fields.groups.data();input.group_count=groups();
    input.prepared=prepared;input.frame_prepared=group_prepared;input.capture_prepared=force_capture.prepared;
    const auto capture=force_capture.buffer();
    input.acceleration_xyz=capture.acceleration_xyz;input.angular_acceleration_xyz=capture.angular_acceleration_xyz;
    input.group_acceleration=capture.groups;input.acceleration_nodes=capture.capacity_nodes;
    input.acceleration_groups=capture.capacity_groups;
    const auto r=Convert(observation::ObserveForceStage(input,&next.force_stage));if(!r)return r;
    // This marks only the private candidate. The accepted accessor changes
    // after the common owner/history commit selects this whole sample.
    next.has_force_stage=true;return Success();
}
} // namespace crash::cases::source_assembly_dynamics
