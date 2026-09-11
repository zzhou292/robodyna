#include "Internal.h"
#include <type_traits>

namespace crash::cases::vehicle_startup::shell_binding_detail {
namespace {
template<class T> auto Decode(const modelio::vehicle::source::CanonicalData& source,const char* name) {
    const auto& array=modelio::vehicle::source::FindArray(source,name);
    return output::arrays::Decode<T>(array.descriptor,array.bytes);
}
template<class Row,class Input> Row Parent(const Input& input,const ReferenceRow& source,
    const std::vector<std::uint64_t>& ids,const std::vector<std::uint32_t>& connections,
    const std::vector<std::size_t>& mapping) {
    Row row;row.reference=input;row.source_parent_id=source.element_id;
    for(std::size_t n=0;n<row.nodes.size();++n) {
        const auto canonical=connections.at(4*source.canonical_parent+n);
        row.nodes[n]=mapping.at(canonical);
        output::Require(row.nodes[n]!=SIZE_MAX && ids.at(canonical)==input.node_ids[n],
            "Shell reference node differs from its canonical source association");
    }
    return row;
}
}
tl::fea::ShellFormulationCollectionInput Inputs::Borrow(std::size_t nodes) const noexcept {
    return {{qeph.empty()?nullptr:qeph.data(),t3.empty()?nullptr:t3.data(),qeph.size(),t3.size(),nodes},
            qbat.empty()?nullptr:qbat.data(),qbat.size()};
}
Inputs Pack(const VehicleShellReferences& refs) {
    const auto& source=refs.source();const auto& canonical=source.canonical().data();
    const auto ids=Decode<std::uint64_t>(canonical,"node_ids");
    const auto connections=Decode<std::uint32_t>(canonical,"shells_node_indices");
    output::Require(ids.size()==canonical.canonical_nodes &&
        connections.size()==4*canonical.canonical_shells,"Canonical shell packing extent differs");
    std::vector<std::size_t> mapping(canonical.canonical_nodes,SIZE_MAX);
    for(std::size_t n=0;n<source.canonical_nodes().size();++n) {
        const auto c=source.canonical_nodes()[n];
        output::Require(mapping.at(c)==SIZE_MAX,"Repeated canonical shell-local node");mapping[c]=n;
    }
    Inputs out;const auto& count=refs.counts();
    out.qeph.reserve(count.qeph_succeeded);out.t3.reserve(count.t3_succeeded);out.qbat.reserve(count.qbat_succeeded);
    for(std::size_t e=0;e<refs.rows().size();++e) {
        const auto& row=refs.rows()[e];const auto& parent=source.parents().at(e);
        const auto* native=refs.resolution()->native_mapping(e);
        output::Require(native && row.status==ReferenceStatus::Success && row.canonical_parent==parent.canonical_parent &&
            row.part_index==parent.part_index && row.part_id==source.parts().at(parent.part_index).part_id &&
            row.role==refs.resolution()->role(parent.part_index) &&
            row.rigid_root_index==refs.resolution()->rigid_root_index(parent.part_index),
            "Complete shell source row or ownership role differs");
        const auto append=[&](auto& rows,const auto& input,tl::fea::ShellBindingFamily family) {
            output::Require(native->family==family && native->family_index==rows.size() &&
                row.reference_index==rows.size(),"Shell source family index differs");
            using R=typename std::decay_t<decltype(rows)>::value_type;
            rows.push_back(Parent<R>(input,row,ids,connections,mapping));
        };
        if(const auto* q=refs.qeph(e)) append(out.qeph,q->input,tl::fea::ShellBindingFamily::Qeph);
        else if(const auto* t=refs.t3(e)) append(out.t3,t->input,tl::fea::ShellBindingFamily::T3);
        else {
            const auto* b=refs.qbat(e);output::Require(b,"Missing complete native shell reference");
            auto packed=Parent<tl::fea::ShellQephBindingInput>(b->input().quadrilateral,row,ids,connections,mapping);
            output::Require(native->family==tl::fea::ShellBindingFamily::Qbat && native->family_index==out.qbat.size() &&
                row.reference_index==out.qbat.size(),"QBAT source family index differs");
            out.qbat.push_back({b->input(),packed.nodes,row.element_id});
        }
    }
    return out;
}
}
