#include "SourceAssemblyQephSpinInternal.h"
#include "lib_src/elements/qeph/QephHistory.h"

namespace crash::cases::source_assembly_observation::spin_detail {
Report Select(const SourceAssemblyBindings& b,std::uint64_t id,Selection& next) noexcept {
    if(!id||!b.shells().prepared())return {Status::InvalidInput,"Spin probe requires a source node"};
    const auto& source=b.source().data();
    for(std::size_t n=0;n<source.nodes.size();++n)if(source.nodes[n].source_id==id)next.node=n;
    if(next.node==SIZE_MAX||next.node>=b.shells().node_count()||b.shells().nodes()[next.node].source_id!=id)
        return {Status::WrongIdentity,"Spin source node is absent or misbound"};
    if(const auto* connectors=b.connectors())for(std::size_t e=0;e<2*connectors->connection_count();++e)
        if(connectors->endpoint_mass()[e].global_node==next.node)
            return {Status::InvalidInput,"Spin probe requires a node without connector mass or couple contributions",SIZE_MAX,next.node};
    if(const auto* groups=b.rigid_groups())for(std::size_t m=0;m<groups->member_count();++m)
        if(groups->members()[m].global_node==next.node)
            return {Status::InvalidInput,"Spin probe currently requires an ordinary node",SIZE_MAX,next.node};
    for(const auto& p:source.parents)for(unsigned l=0;l<p.arity;++l)if(p.nodes[l]==next.node) {
        if(p.family!=modelio::assembly::ShellFamily::Qeph||next.count==MaxSpinParents)
            return {Status::InvalidInput,"Spin probe cannot omit incident T3 or excess parents",SIZE_MAX,next.node};
        if(p.index>=source.parents.size()||p.family_index>=b.shells().qeph_count()||
           b.shells().qeph_source_id(p.family_index)!=p.source_id||b.shells().qeph_nodes(p.family_index)[l]!=next.node)
            return {Status::WrongIdentity,"Spin parent source/native connectivity differs"};
        next.parents[next.count]=p.index;next.locals[next.count++]=l;
    }
    if(!next.count)return {Status::WrongIdentity,"Spin node has no native parent"};
    return detail::Success();
}
Report Check(const QephSpinInput& in,const void* out,std::size_t bytes,Selection& selected) noexcept {
    if(!in.bindings||!out||!detail::Range(&in,sizeof(in),out,bytes)||!detail::Range(in.bindings,sizeof(*in.bindings),out,bytes))
        return {Status::InvalidInput,"Invalid spin input/output descriptors"};
    const auto& b=*in.bindings;const auto n=b.shells().node_count(),p=b.shells().qeph_count();
    if(!n||n>fe::MaxNodalStateNodes||in.before.node_count!=n||in.after.node_count!=n||in.parent_count!=p||!p||p>1024)
        return {Status::InvalidInput,"Spin node/parent extents differ before borrowed reads"};
    for(const double* a:{in.before.position_xyz,in.before.velocity_xyz,in.before.angular_velocity_xyz,
                        in.applied_force_xyz,in.applied_couple_xyz,
                        in.after.position_xyz,in.after.velocity_xyz,in.after.angular_velocity_xyz})
        if(!detail::Range(a,3*n*sizeof(double),out,bytes))return {Status::InvalidInput,"Spin field output overlaps or overflows"};
    if(!detail::Range(in.before.orientation_wxyz,4*n*sizeof(double),out,bytes)||
       !detail::Range(in.parents,p*sizeof(*in.parents),out,bytes)||!detail::Range(in.sections,p*sizeof(*in.sections),out,bytes)||
       !detail::Range(in.parent_diagnostics,sizeof(*in.parent_diagnostics),out,bytes)||
       !detail::Range(in.candidate_parents,p*sizeof(*in.candidate_parents),out,bytes)||
       !detail::Range(in.candidate_sections,p*sizeof(*in.candidate_sections),out,bytes)||
       !detail::Range(in.candidate_diagnostics,sizeof(*in.candidate_diagnostics),out,bytes))
        return {Status::InvalidInput,"Spin parent output overlaps or overflows"};
    rigid::ObservationPhase before,after;auto r=detail::CheckPrepared(b,in.base,in.prepared,n,before,after);if(!r)return r;
    if(in.base.epoch==UINT64_MAX||!std::isfinite(in.prepared.proposed_time)||in.prepared.proposed_time<=in.base.time)
        return {Status::InvalidPhase,"Spin enclosing interval does not advance"};
    const auto& d=*in.parent_diagnostics;
    if(!d.valid||d.phase!=fe::qeph::BatchPhase::Accepted||d.usage!=fe::qeph::BatchUsage::CoupledForces||
       !in.configuration_id||!in.qualification_id||d.configuration_id!=in.configuration_id||d.qualification_id!=in.qualification_id||
       d.owner_id!=in.base.owner_id||d.epoch!=in.base.epoch||d.time!=in.base.time||d.velocity_time!=in.base.velocity_time)
        return {Status::WrongIdentity,"Spin packets do not identify the accepted owner"};
    const auto& c=*in.candidate_diagnostics;const auto& v=in.prepared;
    if(!c.valid||c.phase!=fe::qeph::BatchPhase::Prepared||c.usage!=d.usage||c.owner_id!=d.owner_id||
       c.configuration_id!=d.configuration_id||c.qualification_id!=d.qualification_id||
       c.base_epoch!=in.base.epoch||c.epoch!=in.base.epoch+1||c.attempt!=v.attempt||
       c.time!=v.proposed_time||c.velocity_time!=v.velocity_time||c.base_time!=v.base_time||
       c.base_velocity_time!=v.base_velocity_time||c.kick_dt!=v.kick_dt)
        return {Status::WrongIdentity,"Spin candidate packets have a different prepared association"};
    return Select(b,in.source_node,selected);
}
}
namespace crash::cases::source_assembly_observation {
Report CheckQephSpinSource(const SourceAssemblyBindings& b,std::uint64_t n) noexcept {
    spin_detail::Selection selection;return spin_detail::Select(b,n,selection);
}
}
