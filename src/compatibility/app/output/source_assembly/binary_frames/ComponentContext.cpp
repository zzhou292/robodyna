#include "ComponentContext.h"

namespace crash::output::assembly::binary {
records::Context ComponentContext(const cases::source_assembly::SourceAssemblyBindings& bindings,
        const SourceAssemblySurface& surface,std::uint64_t configuration,std::uint64_t qualification,
        double fixed_dt,records::RecordLimits limits) {
    const auto& src=bindings.source().data();const auto& b=surface.binding();
    CheckWallSourceSchema(src.schema);
    Require(src.identity.sha256==surface.source().data().identity.sha256&&
        src.identity.bytes==surface.source().data().identity.bytes&&
        b.vertices.size()==src.nodes.size()&&surface.parents().size()==src.parents.size()&&
        bindings.materials().Matches(bindings.shells()),"Component frame source/catalog mismatch");
    Require(src.nodes.size()<=limits.nodes&&src.parents.size()<=limits.parents&&
        src.parents.size()<=limits.points/3,"Component frame record capacity exceeded");
    for(std::size_t i=0;i<b.vertices.size();++i)
        Require(b.vertices[i].source.instance==bindings.source_instance_id()&&
            b.vertices[i].source.node==src.nodes[i].source_id&&b.vertices[i].tl_node==i,
            "Component source node/instance mapping differs");
    std::vector<records::ParentPoints> parents;parents.reserve(surface.parents().size());
    for(const auto& p:surface.parents()) {
        const auto family=p.family==source::ShellFamily::Qeph?tl::fea::ShellBindingFamily::Qeph:
            tl::fea::ShellBindingFamily::T3;
        tl::fea::ShellSectionLaw law;
        Require(bindings.materials().Law(family,p.family_index,&law)&&
            law==tl::fea::ShellSectionLaw::LayeredLaw44Nip3,
            "Component binary producer requires actual LAW44 NIP3 native sections");
        const auto* native=bindings.materials().parent(p.source_index);
        Require(native&&native->family==family&&native->family_index==p.family_index&&
            native->source_parent_id==p.element&&native->source_part_id==p.part&&
            native->material_id==p.material&&native->section_id==p.section,
            "Component native material/source parent mapping differs");
        parents.push_back({p.element,p.part,p.source_elform,
            p.family==source::ShellFamily::Qeph?QephFamily:T3Family,3,
            records::PlasticField::NativeEquivalentPlasticStrain});
    }
    records::Identity id{b.identity.owner,b.identity.run,b.identity.topology,
        bindings.source_instance_id(),configuration,qualification,src.identity.bytes,
        src.identity.sha256,ComponentMappingDigest(surface)};
    return records::Context::Create(std::move(id),src.nodes.size(),parents.data(),parents.size(),fixed_dt,limits);
}
} // namespace crash::output::assembly::binary
