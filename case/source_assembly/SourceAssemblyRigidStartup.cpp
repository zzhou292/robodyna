#include "SourceAssemblyStartupInternal.h"
#include "output/ArtifactIO.h"
#include <sstream>

namespace crash::cases::source_assembly::startup {
void RigidGroups(const source::Data& data,const tl::fea::ShellBatchBinding& binding,
                 const SourceAssemblyBindingOptions& options,tl::fea::NodalRigidGroupModel& model) {
    std::size_t count=0,member_count=0;
    for(const auto& group:data.nodal_rigid_groups) if(group.internal) { ++count; member_count+=group.members.size(); }
    if(!count) return;
    std::vector<tl::fea::NodalRigidGroupMember> members; members.reserve(member_count);
    std::vector<tl::fea::NodalRigidGroupInput> groups; groups.reserve(count);
    for(const auto& group:data.nodal_rigid_groups) {
        if(!group.internal) continue; // Explicitly retained released connections never enter active groups.
        output::Require(group.external_nodes.empty()&&group.members.size()==group.selected_global_nodes.size(),
                        "Incomplete internal source rigid group");
        const auto offset=members.size();
        for(std::size_t local=0;local<group.members.size();++local) {
            const auto global=group.selected_global_nodes[local];
            output::Require(global<binding.node_count()&&global<data.nodes.size(),"Rigid source node index is outside the native collection");
            const auto& node=binding.nodes()[global]; const auto& source=data.nodes[global];
            // Prove identity and represented coordinates before ANY group startup.
            output::Require(node.source_id==group.members[local]&&source.source_id==node.source_id&&
                output::Bits(source.position_m.x)==output::Bits(node.position.x)&&
                output::Bits(source.position_m.y)==output::Bits(node.position.y)&&
                output::Bits(source.position_m.z)==output::Bits(node.position.z),"Rigid member source/native identity changed");
            const auto& native=node.native;
            members.push_back({node.source_id,global,node.position,native.mass,native.isotropic_inertia,
                               native.physical_inertia,native.added_inertia});
        }
        groups.push_back({group.id,group.node_set_id,members.data()+offset,group.members.size()});
    }
    // reserve(total) above keeps every borrowed group range stable until this
    // owning TL initializer copies it. Generated primary mass/J stay in TL's
    // regularization ledger and never append a physical source node here.
    const auto report=model.Initialize({options.source_instance_id,data.nodes.size(),groups.data(),groups.size(),
        {data.units.mass_to_kg,data.units.length_to_m},options.rigid_limits});
    if(!report) {
        std::ostringstream message;
        message<<"Assembly rigid startup: group_index="<<report.group<<" member_index="<<report.member
               <<" source_group_id="<<(report.group<groups.size()?groups[report.group].source_group_id:0)
               <<" status="<<unsigned(report.status)<<": "<<report.message;
        throw SourceAssemblyBindingError(SourceAssemblyBindingStage::RigidGroups,unsigned(report.status),message.str());
    }
}
} // namespace crash::cases::source_assembly::startup
