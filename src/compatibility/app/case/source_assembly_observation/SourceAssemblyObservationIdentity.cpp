#include "SourceAssemblyObservationInternal.h"

namespace crash::cases::source_assembly_observation::detail {
Report CheckFrame(const SourceAssemblyBindings& b, fe::HostNodalKinematicsView v,
                  const fe::NodalRigidGroupSnapshot* groups, std::size_t count,
                  const void* out, std::size_t out_bytes) noexcept {
    const auto* model=b.rigid_groups();
    const auto nodes=b.shells().node_count();
    if(!out || !model || !nodes || nodes>fe::MaxNodalStateNodes ||
       !count || count>MaxGroups || count!=model->group_count() || v.node_count!=nodes)
        return {Status::InvalidInput,"Incomplete bounded source observation"};
    if(!Range(v.velocity_xyz,3*nodes*sizeof(double),out,out_bytes) ||
       !Range(v.angular_velocity_xyz,3*nodes*sizeof(double),out,out_bytes) ||
       !Range(groups,count*sizeof(*groups),out,out_bytes) ||
       !Range(&b,sizeof(b),out,out_bytes))
        return {Status::InvalidInput,"Observation ranges overlap or overflow"};
    for(std::size_t g=0;g<count;++g) {
        const auto& p=model->groups()[g];
        if(p.member_count<3 || p.member_count>MaxMembers ||
           groups[g].source_group_id!=p.source_group_id || groups[g].source_node_set_id!=p.source_node_set_id)
            return {Status::WrongIdentity,"Rigid observation group/source order differs",g};
    }
    return Success();
}
Report AcceptedPhase(const fe::NodalStamp& stamp, rigid::ObservationPhase& phase) noexcept {
    if(!stamp.owner_id || !stamp.has_rotations || !std::isfinite(stamp.fixed_dt) || stamp.fixed_dt<=0 ||
       !std::isfinite(stamp.time) || !std::isfinite(stamp.velocity_time) ||
       !std::isfinite(stamp.reaction_time) || !std::isfinite(stamp.reaction_kick_dt) ||
       stamp.temporal_scheme!=fe::NodalTemporalScheme::StaggeredHalfKickStart)
        return {Status::InvalidPhase,"Observation requires a staggered extended owner"};
    if(stamp.epoch==0) {
        if(stamp.time!=0 || stamp.velocity_time!=0 || stamp.reactions_valid ||
           stamp.reaction_base_epoch || stamp.reaction_time!=0 || stamp.reaction_kick_dt!=0 ||
           stamp.velocity_phase!=fe::NodalVelocityPhase::Collocated)
            return {Status::InvalidPhase,"Initial sample is not physical initialization"};
        phase={rigid::ObservationPhaseKind::PhysicalInitialization,0,0,0};
    } else {
        if(!stamp.reactions_valid || stamp.reaction_base_epoch!=stamp.epoch-1 || stamp.reaction_time<0 ||
           stamp.time!=stamp.reaction_time+stamp.fixed_dt ||
           stamp.velocity_time!=stamp.reaction_time+.5*stamp.fixed_dt ||
           !(stamp.time>stamp.velocity_time && stamp.velocity_time>stamp.reaction_time) ||
           stamp.reaction_kick_dt!=(stamp.epoch==1 ? .5*stamp.fixed_dt : stamp.fixed_dt) ||
           stamp.velocity_phase!=fe::NodalVelocityPhase::PreviousMidpoint)
            return {Status::InvalidPhase,"Accepted midpoint/frame provenance is incomplete"};
        phase={rigid::ObservationPhaseKind::StoredMidpointWithLaggedFrame,
               stamp.time,stamp.velocity_time,stamp.reaction_time};
    }
    return Success();
}
Report Convert(const rigid::ObservationReport& r, std::size_t g,
               const fe::NodalRigidGroupModel& model) noexcept {
    const auto& group=model.groups()[g];
    const auto n=r.member<group.member_count ? model.members()[group.member_offset+r.member].global_node : SIZE_MAX;
    const auto status=r.status==rigid::ObservationStatus::KickMismatch ? Status::KickMismatch :
        r.status==rigid::ObservationStatus::UnsupportedPhase ? Status::InvalidPhase :
        r.status==rigid::ObservationStatus::NonfiniteResult ? Status::NonfiniteResult : Status::InvalidMetric;
    return {status,"Native rigid value observation failed",g,n,r.dof,r.residual,r.roundoff_budget};
}
Report CheckPrepared(const SourceAssemblyBindings& bindings,const fe::NodalStamp& base,
                     const fe::NodalPreparedView& p,std::size_t nodes,
                     rigid::ObservationPhase& before,rigid::ObservationPhase& after) noexcept {
    if(base.node_count!=nodes || !Scope(bindings,base.rigid_groups) ||
       !Scope(bindings,p.rigid_groups) || p.owner_id!=base.owner_id || !p.attempt ||
       p.kinematics.node_count!=base.node_count || p.base_kinematics.node_count!=base.node_count ||
       p.kinematics.base_epoch!=base.epoch || p.base_kinematics.base_epoch!=base.epoch)
        return {Status::WrongIdentity,"Candidate source/owner observation differs"};
    auto report=AcceptedPhase(base,before); if(!report) return report;
    if(p.temporal_scheme!=base.temporal_scheme || p.base_velocity_phase!=base.velocity_phase ||
       p.velocity_phase!=fe::NodalVelocityPhase::PreviousMidpoint || p.base_time!=base.time ||
       p.base_velocity_time!=base.velocity_time || p.proposed_time!=base.time+base.fixed_dt ||
       p.velocity_time!=base.time+.5*base.fixed_dt || p.kick_dt!=(base.epoch ? base.fixed_dt : .5*base.fixed_dt))
        return {Status::InvalidPhase,"Candidate phase differs from the owner schedule"};
    after={rigid::ObservationPhaseKind::StoredMidpointWithLaggedFrame,p.proposed_time,p.velocity_time,p.base_time};
    return Success();
}
} // namespace crash::cases::source_assembly_observation::detail
