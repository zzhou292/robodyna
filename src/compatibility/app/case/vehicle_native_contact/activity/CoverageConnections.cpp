#include "Coverage.h"
#include "modelio/type45/SourcePolicy.h"
namespace crash::cases::vehicle_native_contact::activity::coverage {
void Connections(const detail::SourceInputs& in,Coverage& coverage) {
    const auto& mechanical=in.owner.execution_source().mechanical();
    const auto& welds=mechanical.welds();const auto& declared=welds.source().data().spotwelds;
    const auto& model=welds.model();
    output::Require(model.connection_count()==declared.size(),"Activity source excludes an original executed spotweld");
    for(std::size_t i=0;i<declared.size();++i) {
        const auto& actual=model.connections()[i];const auto& source=declared[i];
        output::Require(actual.source_element_id==source.id,"Spotweld support source order differs");
        for(unsigned k=0;k<2;++k)output::Require(actual.source_node_id[k]==source.nodes[k],"Spotweld support endpoint differs");
    }
    const auto& joint_source=in.owner.joint_source();const auto& joints=in.owner.joints();
    output::Require(joint_source.policy()==modelio::type45::Policy::OriginalDirectSdiType45NativeSupportsV6&&
        joint_source.source_domain().policy()==modelio::physical_domain::Policy::RetainedShellAssembliesNativeSupportsV6&&
        joint_source.data().boundaries==0&&joint_source.data().required==modelio::type45::detail::Required(joint_source.policy())&&
        joints.joints().size()==joint_source.data().required&&in.owner.joint_source_rows().size()==joints.joints().size(),
        "Activity support requires the complete V6 joint source, not historical assembly-boundary omissions");
    std::vector<std::uint8_t> seen(joint_source.data().rows.size());
    for(std::size_t i=0;i<joints.joints().size();++i) {
        const auto row=in.owner.joint_source_rows()[i];
        output::Require(row<seen.size()&&!seen[row],"Joint support source row is missing or duplicated");seen[row]=1;
        const auto& source=joint_source.data().rows[row];const auto geometry=source.Geometry();const auto& actual=joints.joints()[i];
        output::Require(source.disposition==modelio::type45::Disposition::Required&&actual.geometry.source_joint_id==source.source_id,
            "Joint support is not the declared actual source joint");
        for(unsigned k=0;k<2;++k)output::Require(actual.geometry.source_node_id[k]==geometry.source_node_id[k]&&
            mechanical.domain().nodes()[actual.domain_nodes[k]].source_id==geometry.source_node_id[k],"Joint physical endpoint differs");
    }
    for(auto value:seen)output::Require(value==1,"An original V6 joint was omitted from node support");
    coverage.welds=model.connection_count();coverage.joints=joints.joints().size();
}
}
