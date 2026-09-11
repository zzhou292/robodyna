#include "ReferenceStorage.h"

namespace crash::cases::vehicle_startup::detail {
namespace {
namespace source=modelio::vehicle::source;
template<class T> std::vector<T> Decode(const source::CanonicalData& d,const char* name) {
    const auto& a=source::FindArray(d,name);return output::arrays::Decode<T>(a.descriptor,a.bytes);
}
template<class Input,std::size_t Arity>
Input InputFor(const Geometry& g,std::size_t canonical,const modelio::assembly::Material& material,
               const modelio::assembly::Section& section,tl::fea::ShellReferencePlacement placement) {
    modelio::assembly::SourceReferenceNode nodes[Arity];
    for(std::size_t n=0;n<Arity;++n) {
        const auto i=g.connections.at(4*canonical+n);
        nodes[n]={g.node_ids.at(i),{g.positions.at(3*i),g.positions.at(3*i+1),g.positions.at(3*i+2)}};
        output::Require(nodes[n].source_id==g.records.at(6*canonical+2+n),"Reference source node association changed");
    }
    auto input = modelio::assembly::PackShellReference<Input>(nodes,material,section);
    input.placement = placement;
    return input;
}
}
Geometry::Geometry(const source::CanonicalData& d)
    :node_ids(Decode<std::uint64_t>(d,"node_ids")),records(Decode<std::uint64_t>(d,"shells_records")),
     positions(Decode<double>(d,"node_positions")),connections(Decode<std::uint32_t>(d,"shells_node_indices")),
     lines(Decode<std::uint32_t>(d,"shells_source_lines")) {}
void PrepareRows(const DeclarationView& declarations,const Geometry& g,ReferenceStorage& out) {
    const auto& source=declarations.source;
    for(const auto& parent:source.parents()) {
        const auto c=parent.canonical_parent;const auto& part=source.parts().at(parent.part_index);
        ReferenceRow row;row.element_id=g.records.at(6*c);row.part_id=part.part_id;
        row.material_id=part.material_id;row.section_id=part.section_id;
        row.source_line=g.lines.at(c);row.canonical_parent=c;row.part_index=parent.part_index;
        output::Require(row.part_id==g.records.at(6*c+1),"Reference source part association changed");
        const auto* m=declarations.Material(parent.part_index);
        const auto* s=declarations.Section(parent.part_index);
        if (!m && !s) {
            AppendUnresolved(out,row);
            continue;
        }
        output::Require(m && s,"Supported reference lacks typed source declaration");
        const auto placement = declarations.Placement(parent.part_index);
        if(g.records.at(6*c+4)!=g.records.at(6*c+5))
            Append(out,row,InputFor<tl::fea::qeph::ReferenceInput,4>(g,c,*m,*s,placement));
        else Append(out,row,InputFor<tl::fea::t3::ReferenceInput,3>(g,c,*m,*s,placement));
    }
}
} // namespace crash::cases::vehicle_startup::detail
