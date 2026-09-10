#include "SourceAssemblyForceStageInternal.h"

namespace crash::cases::source_assembly_observation {
namespace {
Report Groups(const ForceStageInput& in,ForceStageSummary& next,detail::Membership& membership) noexcept {
    const auto& model=*in.bindings->rigid_groups();
    std::array<rigid::MemberMotion,detail::MaxMembers> motion;
    std::array<rigid::ForceStageAcceleration,detail::MaxMembers> acceleration;
    long double replacement=0;
    for(std::size_t g=0;g<in.group_count;++g) {
        const auto& p=model.groups()[g];
        for(std::size_t i=0;i<p.member_count;++i) {
            const auto& member=model.members()[p.member_offset+i]; const auto n=member.global_node;
            if(n>=in.before.node_count || membership[n])
                return {Status::InvalidMetric,"Overlapping force-stage source membership",g,n};
            const auto& node=in.bindings->shells().nodes()[n]; const auto& m=node.native;
            if(member.source_node_id!=node.source_id || member.mass_kg!=m.mass ||
               member.total_inertia_kg_m2!=m.isotropic_inertia || member.physical_inertia_kg_m2!=m.physical_inertia ||
               member.added_inertia_kg_m2!=m.added_inertia)
                return {Status::InvalidMetric,"Force-stage native source coefficients differ",g,n};
            membership[n]=true; motion[i]=detail::Motion(in.before,n);
            acceleration[i]={detail::Vector(in.acceleration_xyz,n),detail::Vector(in.angular_acceleration_xyz,n)};
        }
        const auto& before=in.before_groups[g].state; const auto& a=in.group_acceleration[g];
        const rigid::GroupForceStageKineticInput input{{&p,model.members()+p.member_offset,p.member_count},
            motion.data(),acceleration.data(),{before.velocity,before.omega},{a.acceleration,a.angular_acceleration},
            in.force_groups[g].state.principal_axes,next.phase};
        rigid::GroupForceStageKineticObservation observed;
        const auto report=rigid::ObserveGroupForceStageKinetic(input,observed);
        if(!report) return detail::Convert(report,g,model);
        detail::Add(next.grouped_members,observed.members); detail::Add(next.groups,observed.aggregate);
        replacement+=observed.replacement;
    }
    next.replacement=replacement;
    return detail::Success();
}
Report Ordinary(const ForceStageInput& in,const detail::Membership& members,ForceStageSummary& next) noexcept {
    detail::NativeKineticSums sum{};
    const double half=.5*next.phase.durations.previous_drift_dt;
    for(std::size_t n=0;n<in.before.node_count;++n) if(!members[n]) {
        rigid::MemberMotion collocated;
        const rigid::ForceStageAcceleration acceleration{detail::Vector(in.acceleration_xyz,n),
                                                         detail::Vector(in.angular_acceleration_xyz,n)};
        if(!rigid::force_stage_detail::Collocate(detail::Motion(in.before,n),acceleration,half,collocated))
            return {Status::NonfiniteResult,"Nonfinite ordinary force-stage motion",SIZE_MAX,n};
        const auto report=detail::AddNativeMotion(in.bindings->shells().nodes()[n].native,collocated,n,sum);
        if(!report) return report;
    }
    detail::AssignNative(sum,next.ordinary);
    return detail::Success();
}
}
Report ObserveForceStage(const ForceStageInput& input,ForceStageSummary* output) noexcept {
    ForceStageSummary next;
    auto report=detail::CheckForceStage(input,output,next.phase); if(!report) return report;
    next.owner_id=input.base.owner_id; next.source=input.base.rigid_groups;
    next.base_epoch=input.base.epoch; next.attempt=input.prepared.attempt;
    next.enclosing_epoch=input.base.epoch+1; next.enclosing_time=input.prepared.proposed_time;
    detail::Membership membership{};
    report=Groups(input,next,membership); if(!report) return report;
    report=Ordinary(input,membership,next); if(!report) return report;
    next.native_total=next.ordinary.total+next.grouped_members.total;
    next.effective_total=next.ordinary.total+next.groups.total;
    if(!detail::Finite(next.ordinary) || !detail::Finite(next.grouped_members) || !detail::Finite(next.groups) ||
       !std::isfinite(next.native_total) || !std::isfinite(next.effective_total) || !std::isfinite(next.replacement))
        return {Status::NonfiniteResult,"Force-stage kinetic reduction overflows"};
    *output=next;
    return detail::Success();
}
} // namespace crash::cases::source_assembly_observation
