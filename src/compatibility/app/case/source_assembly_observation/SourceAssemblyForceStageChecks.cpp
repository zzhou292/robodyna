#include "SourceAssemblyForceStageInternal.h"

namespace crash::cases::source_assembly_observation::detail {
Report CheckForceStage(const ForceStageInput& in,ForceStageSummary* out,
                       rigid::ForceStageObservationPhase& phase) noexcept {
    if(!in.bindings || !out || !Range(&in,sizeof(in),out,sizeof(*out)))
        return {Status::InvalidInput,"Missing or overlapping force-stage input"};
    const auto& b=*in.bindings;
    for(const auto* groups:{in.before_groups,in.force_groups}) {
        const auto r=CheckFrame(b,in.before,groups,in.group_count,out,sizeof(*out));
        if(!r) return r;
    }
    const auto nodes=in.before.node_count;
    if(in.acceleration_nodes!=nodes || in.acceleration_groups!=in.group_count)
        return {Status::InvalidInput,"Force-stage capture active counts differ"};
    for(const auto* values:{in.acceleration_xyz,in.angular_acceleration_xyz})
        if(!Range(values,3*nodes*sizeof(double),out,sizeof(*out)))
            return {Status::InvalidInput,"Missing or overlapping force-stage acceleration"};
    const auto& model=*b.rigid_groups();
    if(!Range(in.group_acceleration,in.group_count*sizeof(*in.group_acceleration),out,sizeof(*out)) ||
       !Range(model.groups(),model.group_count()*sizeof(*model.groups()),out,sizeof(*out)) ||
       !Range(model.members(),model.member_count()*sizeof(*model.members()),out,sizeof(*out)) ||
       !Range(b.shells().active_nodes().data(),nodes*sizeof(fe::ShellBindingNode),out,sizeof(*out)))
        return {Status::InvalidInput,"Force-stage output overlaps source or capture ranges"};
    if(!fe::trial_identity::SameStamp(in.base,in.before_group_stamp) ||
       !fe::trial_identity::SamePrepared(in.prepared,in.frame_prepared) ||
       !fe::trial_identity::SamePrepared(in.prepared,in.capture_prepared) || in.base.epoch==UINT64_MAX)
        return {Status::WrongIdentity,"Force-stage readbacks do not share the complete owner identity"};
    rigid::ObservationPhase before,after;
    auto report=CheckPrepared(b,in.base,in.prepared,nodes,before,after); if(!report) return report;
    // Require actual complete borrowed device views as identity values only.
    // They are never dereferenced by this host adapter.
    if(!fe::trial_identity::ValidKinematics(in.prepared.kinematics,nodes,in.base.epoch) ||
       !fe::trial_identity::ValidKinematics(in.prepared.base_kinematics,nodes,in.base.epoch))
        return {Status::WrongIdentity,"Force-stage prepared identity has incomplete source views"};
    phase={in.prepared.base_time,in.base.velocity_time,before.frame_time,
           {in.base.epoch?in.base.fixed_dt:0,in.prepared.kick_dt,in.base.fixed_dt}};
    if(!rigid::force_stage_detail::Phase(phase) || !std::isfinite(in.prepared.proposed_time) ||
       !(in.prepared.proposed_time>in.prepared.velocity_time && in.prepared.velocity_time>in.prepared.base_time))
        return {Status::InvalidPhase,"Force-stage duration or enclosing candidate time is invalid"};
    for(std::size_t g=0;g<in.group_count;++g) {
        const auto& p=model.groups()[g]; const auto& a=in.group_acceleration[g];
        if(a.source_group_id!=p.source_group_id || a.source_node_set_id!=p.source_node_set_id || a.member_count!=p.member_count)
            return {Status::WrongIdentity,"Force-stage acceleration source group order differs",g};
        if(!rigid::ValidGroupState(in.before_groups[g].state) || !rigid::ValidGroupState(in.force_groups[g].state))
            return {Status::InvalidInput,"Force-stage group state/frame is invalid",g};
        if(p.member_offset>model.member_count() || p.member_count>model.member_count()-p.member_offset)
            return {Status::InvalidMetric,"Force-stage source membership range is invalid",g};
    }
    return Success();
}
} // namespace crash::cases::source_assembly_observation::detail
