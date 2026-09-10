#include "SourceAssemblyObservationInternal.h"
#include "lib_src/constraints/NodalRigidKineticObservation.h"
#include <algorithm>

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
namespace {
void Add(rigid::MemberKineticChannels& a,const rigid::MemberKineticChannels& b) {
    for(auto field:{&rigid::MemberKineticChannels::translation,&rigid::MemberKineticChannels::native_rotation,
            &rigid::MemberKineticChannels::physical_rotation,&rigid::MemberKineticChannels::added_rotation,
            &rigid::MemberKineticChannels::total,&rigid::MemberKineticChannels::inertia_partition_residual}) a.*field+=b.*field;
}
void Add(rigid::AggregateKineticChannels& a,const rigid::AggregateKineticChannels& b) {
    for(auto field:{&rigid::AggregateKineticChannels::translation,&rigid::AggregateKineticChannels::rotation,
            &rigid::AggregateKineticChannels::total,&rigid::AggregateKineticChannels::structural_translation,
            &rigid::AggregateKineticChannels::primary_translation,&rigid::AggregateKineticChannels::member_orbital_rotation,
            &rigid::AggregateKineticChannels::native_member_rotation,&rigid::AggregateKineticChannels::physical_member_rotation,
            &rigid::AggregateKineticChannels::added_member_rotation,&rigid::AggregateKineticChannels::primary_parallel_axis_rotation,
            &rigid::AggregateKineticChannels::primary_isotropic_rotation,&rigid::AggregateKineticChannels::principal_correction_rotation,
            &rigid::AggregateKineticChannels::decomposition_residual,&rigid::AggregateKineticChannels::decomposition_roundoff_budget})
        a.*field+=b.*field;
}
}
Report ObserveKinetic(const SourceAssemblyBindings& b,fe::HostNodalKinematicsView v,
                      const fe::NodalRigidGroupSnapshot* groups,rigid::ObservationPhase phase,
                      const fe::ShellBatchKinetic& published,KineticSummary& next,Membership& membership) noexcept {
    const auto& model=*b.rigid_groups();
    next={}; next.phase=phase; membership.fill(false);
    std::array<rigid::MemberMotion,MaxMembers> motion;
    for(std::size_t g=0;g<model.group_count();++g) {
        const auto& p=model.groups()[g];
        for(std::size_t i=0;i<p.member_count;++i) {
            const auto n=model.members()[p.member_offset+i].global_node;
            if(n>=v.node_count || membership[n]) return {Status::InvalidMetric,"Overlapping rigid source membership",g,n};
            membership[n]=true; motion[i]=Motion(v,n);
        }
        rigid::GroupKineticObservation observed;
        const auto report=rigid::ObserveGroupKinetic({{&p,model.members()+p.member_offset,p.member_count},
            motion.data(),groups[g].state,phase},observed);
        if(!report) return Convert(report,g,model);
        Add(next.grouped_members,observed.members); Add(next.groups,observed.aggregate);
    }
    std::array<long double,4> free{},all{};
    for(std::size_t n=0;n<v.node_count;++n) {
        const auto& m=b.shells().nodes()[n].native;
        for(unsigned a=0;a<3;++a) {
            const long double speed=v.velocity_xyz[3*n+a],omega=v.angular_velocity_xyz[3*n+a];
            if(!std::isfinite(speed) || !std::isfinite(omega)) return {Status::NonfiniteResult,"Nonfinite nodal motion",SIZE_MAX,n,a};
            const long double values[]{.5L*m.mass*speed*speed,.5L*m.isotropic_inertia*omega*omega,
                                      .5L*m.physical_inertia*omega*omega,.5L*m.added_inertia*omega*omega};
            for(unsigned c=0;c<4;++c) { all[c]+=values[c]; if(!membership[n]) free[c]+=values[c]; }
        }
    }
    auto& ordinary=next.ordinary;
    ordinary.translation=free[0]; ordinary.native_rotation=free[1];
    ordinary.physical_rotation=free[2]; ordinary.added_rotation=free[3];
    ordinary.total=free[0]+free[1]; ordinary.inertia_partition_residual=free[1]-free[2]-free[3];
    next.native_total=ordinary.total+next.grouped_members.total;
    next.effective_total=ordinary.total+next.groups.total;
    const double supplied[]{published.translation,published.rotation,published.physical_isotropic,published.added_isotropic};
    for(unsigned c=0;c<4;++c) {
        // Independent long-double physical-node sum versus native ordered binary64
        // reduction. This only checks bookkeeping, not the model's physical energy.
        const long double residual=all[c]-supplied[c];
        const long double budget=(32+8*v.node_count)*Epsilon*(std::abs(all[c])+std::abs(supplied[c]));
        if(!std::isfinite(supplied[c]) || supplied[c]<0 || !std::isfinite(static_cast<double>(all[c])) ||
           !std::isfinite(static_cast<double>(budget)) || std::abs(residual)>budget)
            return {Status::KineticMismatch,"Native nodal kinetic publication differs",SIZE_MAX,SIZE_MAX,c,
                    static_cast<double>(residual),static_cast<double>(budget)};
        next.publication_residual=std::max(next.publication_residual,static_cast<double>(std::abs(residual)));
        next.publication_roundoff_budget=std::max(next.publication_roundoff_budget,static_cast<double>(budget));
    }
    if(!std::isfinite(next.native_total) || !std::isfinite(next.effective_total))
        return {Status::NonfiniteResult,"Collection kinetic sum overflows"};
    return Success();
}
} // namespace crash::cases::source_assembly_observation::detail

namespace crash::cases::source_assembly_observation {
Report ObserveInitial(const SourceAssemblyBindings& b,const fe::NodalStamp& stamp,
                      fe::HostNodalKinematicsView v,const fe::NodalRigidGroupSnapshot* groups,
                      std::size_t count,const fe::ShellBatchKinetic& published,KineticSummary* output) noexcept {
    auto report=detail::CheckFrame(b,v,groups,count,output,sizeof(*output)); if(!report) return report;
    if(!detail::Range(&stamp,sizeof(stamp),output,sizeof(*output)) ||
       !detail::Range(&published,sizeof(published),output,sizeof(*output)))
        return {Status::InvalidInput,"Initial observation output overlaps its inputs"};
    if(stamp.epoch || stamp.node_count!=v.node_count || !detail::Scope(b,stamp.rigid_groups))
        return {Status::WrongIdentity,"Initial source/owner observation differs"};
    rigid::ObservationPhase phase; report=detail::AcceptedPhase(stamp,phase); if(!report) return report;
    KineticSummary next; detail::Membership membership;
    report=detail::ObserveKinetic(b,v,groups,phase,published,next,membership); if(!report) return report;
    *output=next; return detail::Success();
}
} // namespace crash::cases::source_assembly_observation
