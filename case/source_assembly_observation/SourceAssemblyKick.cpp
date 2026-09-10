#include "SourceAssemblyObservationInternal.h"
#include "lib_src/constraints/NodalRigidKickObservation.h"

namespace crash::cases::source_assembly_observation {
namespace {
Report CheckInput(const Input& in,Summary* out,rigid::ObservationPhase& before,
                  rigid::ObservationPhase& after) noexcept {
    if(!in.bindings || !out || !detail::Range(&in,sizeof(in),out,sizeof(*out)))
        return {Status::InvalidInput,"Missing or overlapping interval observation"};
    for(const auto pair:{std::pair{in.before,in.before_groups},std::pair{in.after,in.after_groups}}) {
        const auto r=detail::CheckFrame(*in.bindings,pair.first,pair.second,in.group_count,out,sizeof(*out));
        if(!r) return r;
    }
    const auto bytes=3*in.before.node_count*sizeof(double);
    for(const auto* values:{in.applied_force_xyz,in.applied_couple_xyz,in.reaction_force_xyz,in.reaction_couple_xyz})
        if(!detail::Range(values,bytes,out,sizeof(*out))) return {Status::InvalidInput,"Missing or overlapping kick loads"};
    const auto& base=in.base; const auto& p=in.prepared;
    if(base.node_count!=in.before.node_count || !detail::Scope(*in.bindings,base.rigid_groups) ||
       !detail::Scope(*in.bindings,p.rigid_groups) || p.owner_id!=base.owner_id || !p.attempt ||
       p.kinematics.node_count!=base.node_count || p.base_kinematics.node_count!=base.node_count ||
       p.kinematics.base_epoch!=base.epoch || p.base_kinematics.base_epoch!=base.epoch)
        return {Status::WrongIdentity,"Candidate source/owner observation differs"};
    auto report=detail::AcceptedPhase(base,before); if(!report) return report;
    if(p.temporal_scheme!=base.temporal_scheme || p.base_velocity_phase!=base.velocity_phase ||
       p.velocity_phase!=fe::NodalVelocityPhase::PreviousMidpoint || p.base_time!=base.time ||
       p.base_velocity_time!=base.velocity_time || p.proposed_time!=base.time+base.fixed_dt ||
       p.velocity_time!=base.time+.5*base.fixed_dt || p.kick_dt!=(base.epoch ? base.fixed_dt : .5*base.fixed_dt))
        return {Status::InvalidPhase,"Candidate phase differs from the owner schedule"};
    after={rigid::ObservationPhaseKind::StoredMidpointWithLaggedFrame,p.proposed_time,p.velocity_time,p.base_time};
    return detail::Success();
}
struct NativeKick {
    long double delta=0,ordinary_delta=0,applied[2]{},reaction[2]{},budget=0;
};
Report ObserveNativeKick(const Input& in,const detail::Membership& members,NativeKick& sum) noexcept {
    // Independent host bookkeeping of the supplied physical-node endpoint and
    // actual loads. The per-DOF budget includes binary64 endpoint rounding,
    // including sub-ULP kicks and zero-average reversals. No state is advanced.
    for(std::size_t n=0;n<in.before.node_count;++n) {
        const auto& m=in.bindings->shells().nodes()[n].native;
        for(unsigned dof=0;dof<6;++dof) {
            const bool rotation=dof>=3; const auto index=3*n+dof%3;
            const long double coefficient=rotation ? m.isotropic_inertia : m.mass;
            const long double v0=(rotation ? in.before.angular_velocity_xyz : in.before.velocity_xyz)[index];
            const long double v1=(rotation ? in.after.angular_velocity_xyz : in.after.velocity_xyz)[index];
            const long double force=(rotation ? in.applied_couple_xyz : in.applied_force_xyz)[index];
            const long double reaction=(rotation ? in.reaction_couple_xyz : in.reaction_force_xyz)[index];
            const long double impulse=coefficient*(v1-v0),a=in.prepared.kick_dt*force,r=in.prepared.kick_dt*reaction;
            const long double allowance=64*detail::Epsilon*(coefficient*(std::abs(v0)+std::abs(v1))+std::abs(a)+std::abs(r));
            const long double residual=impulse-a-r,average=.5L*v0+.5L*v1;
            if(!std::isfinite(force) || !std::isfinite(reaction) || !std::isfinite(static_cast<double>(allowance)) ||
               !std::isfinite(static_cast<double>(residual)))
                return {Status::NonfiniteResult,"Nonfinite native kick observation",SIZE_MAX,n,dof};
            if(std::abs(residual)>allowance)
                return {Status::KickMismatch,"Native member/ordinary kick impulse differs",SIZE_MAX,n,dof,
                        static_cast<double>(residual),static_cast<double>(allowance)};
            const long double delta=impulse*average,applied=a*average,work=r*average;
            sum.delta+=delta; if(!members[n]) sum.ordinary_delta+=delta;
            sum.applied[rotation]+=applied; sum.reaction[rotation]+=work;
            sum.budget+=allowance*std::abs(average)+32*detail::Epsilon*(std::abs(delta)+std::abs(applied)+std::abs(work));
        }
    }
    return detail::Success();
}
Report GroupDelta(const Input& in,rigid::ObservationPhase before,rigid::ObservationPhase after,
                  long double& delta,long double& replacement) noexcept {
    const auto& model=*in.bindings->rigid_groups();
    std::array<rigid::MemberMotion,detail::MaxMembers> old_motion,new_motion;
    std::array<rigid::Wrench,detail::MaxMembers> applied,reaction;
    for(std::size_t g=0;g<model.group_count();++g) {
        const auto& p=model.groups()[g];
        for(std::size_t i=0;i<p.member_count;++i) {
            const auto n=model.members()[p.member_offset+i].global_node;
            old_motion[i]=detail::Motion(in.before,n); new_motion[i]=detail::Motion(in.after,n);
            applied[i]={detail::Vector(in.applied_force_xyz,n),detail::Vector(in.applied_couple_xyz,n)};
            reaction[i]={detail::Vector(in.reaction_force_xyz,n),detail::Vector(in.reaction_couple_xyz,n)};
        }
        rigid::GroupKickObservation value;
        const auto report=rigid::ObserveGroupKick({{&p,model.members()+p.member_offset,p.member_count},
            old_motion.data(),new_motion.data(),in.before_groups[g].state,in.after_groups[g].state,
            before,after,applied.data(),reaction.data(),in.prepared.kick_dt},value);
        if(!report) return detail::Convert(report,g,model);
        delta+=value.aggregate_delta; replacement+=value.replacement_delta;
    }
    return detail::Success();
}
}
Report ObserveInterval(const Input& in,Summary* output) noexcept {
    rigid::ObservationPhase before,after;
    auto report=CheckInput(in,output,before,after); if(!report) return report;
    Summary next; detail::Membership membership;
    report=detail::ObserveKinetic(*in.bindings,in.before,in.before_groups,before,in.base_kinetic,next.before,membership);
    if(!report) return report;
    report=detail::ObserveKinetic(*in.bindings,in.after,in.after_groups,after,in.kinetic,next.after,membership);
    if(!report) return report;
    NativeKick native; report=ObserveNativeKick(in,membership,native); if(!report) return report;
    long double group_delta=0,replacement=0;
    report=GroupDelta(in,before,after,group_delta,replacement); if(!report) return report;
    next.native_delta=native.delta; next.effective_delta=native.ordinary_delta+group_delta;
    // Preserve small group corrections independently of large ordinary-node work.
    next.replacement_delta=replacement;
    next.applied={static_cast<double>(native.applied[0]),static_cast<double>(native.applied[1]),
                  static_cast<double>(native.applied[0]+native.applied[1])};
    next.reaction={static_cast<double>(native.reaction[0]),static_cast<double>(native.reaction[1]),
                   static_cast<double>(native.reaction[0]+native.reaction[1])};
    next.native_residual=(next.native_delta-next.applied.total)-next.reaction.total;
    next.effective_residual=((next.effective_delta-next.applied.total)-next.reaction.total)-next.replacement_delta;
    next.roundoff_budget=native.budget+32*detail::Epsilon*(std::abs(static_cast<long double>(next.native_delta))+
        std::abs(static_cast<long double>(next.effective_delta))+std::abs(static_cast<long double>(next.replacement_delta))+
        std::abs(static_cast<long double>(next.applied.total))+std::abs(static_cast<long double>(next.reaction.total)));
    for(const auto value:{next.native_delta,next.effective_delta,next.replacement_delta,next.applied.translation,
            next.applied.rotation,next.applied.total,next.reaction.translation,next.reaction.rotation,next.reaction.total,
            next.native_residual,next.effective_residual,next.roundoff_budget})
        if(!std::isfinite(value)) return {Status::NonfiniteResult,"Collection kick reduction overflows"};
    const double residual=std::max(std::abs(next.native_residual),std::abs(next.effective_residual));
    if(residual>next.roundoff_budget)
        return {Status::KickMismatch,"Collection kick bookkeeping differs",SIZE_MAX,SIZE_MAX,6,residual,next.roundoff_budget};
    *output=next; return detail::Success();
}
} // namespace crash::cases::source_assembly_observation
