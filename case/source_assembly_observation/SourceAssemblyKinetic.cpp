#include "SourceAssemblyKineticChannels.h"
#include "lib_src/constraints/NodalRigidKineticObservation.h"
#include <algorithm>

namespace crash::cases::source_assembly_observation::detail {
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
    NativeKineticSums free{},all{};
    for(std::size_t n=0;n<v.node_count;++n) {
        const auto report=AddNativeMotion(b.shells().nodes()[n].native,Motion(v,n),n,all,membership[n]?nullptr:&free);
        if(!report) return report;
    }
    auto& ordinary=next.ordinary; AssignNative(free,ordinary);
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
