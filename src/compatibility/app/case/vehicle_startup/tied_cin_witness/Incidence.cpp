#include "Internal.h"
#include "lib_src/assembly/NodalDomainIdentity.h"
#include <algorithm>

namespace crash::cases::vehicle_startup::cin_witness_detail {
namespace {
struct Parent {
    tl::fea::ShellBindingFamily family = tl::fea::ShellBindingFamily::None;
    std::array<std::size_t,4> nodes{};
    unsigned arity = 0;
};
Parent Read(const tl::fea::ShellBatchBinding& binding,const ReferenceRow& row) {
    using Family=ReferenceFamily;
    Parent out;
    std::uint64_t id=0;
    if (row.family==Family::Qeph && row.reference_index<binding.qeph_count()) {
        out.family=tl::fea::ShellBindingFamily::Qeph;
        out.nodes=binding.qeph_nodes(row.reference_index);
        out.arity=4;
        id=binding.qeph_source_id(row.reference_index);
    } else if (row.family==Family::T3 && row.reference_index<binding.t3_count()) {
        out.family=tl::fea::ShellBindingFamily::T3;
        const auto& nodes=binding.t3_nodes(row.reference_index);
        out.nodes={nodes[0],nodes[1],nodes[2],nodes[2]};
        out.arity=3;
        id=binding.t3_source_id(row.reference_index);
    } else if (row.family==Family::Qbat && row.reference_index<binding.qbat_count()) {
        out.family=tl::fea::ShellBindingFamily::Qbat;
        out.nodes=binding.qbat_nodes(row.reference_index);
        out.arity=4;
        id=binding.qbat_source_id(row.reference_index);
    }
    output::Require(out.arity && id==row.element_id && row.status==ReferenceStatus::Success,
                    "CIN witness native parent identity differs from source reference");
    return out;
}
bool Contains(const Parent& parent,const std::array<std::size_t,4>& patch) {
    for (const auto node:patch) {
        if (std::find(parent.nodes.begin(),parent.nodes.end(),node)==parent.nodes.end()) return false;
    }
    return true;
}
std::size_t Find(const tl::fea::ShellBatchBinding& binding,std::uint64_t id) {
    // Complete source binding node order is ascending original NID. No second
    // source/coordinate parser or independently sorted owner map is created.
    std::size_t first=0,last=binding.node_count();
    while (first<last) {
        const auto middle=first+(last-first)/2;
        if (binding.nodes()[middle].source_id<id) first=middle+1;
        else last=middle;
    }
    return first<binding.node_count() && binding.nodes()[first].source_id==id ? first : SIZE_MAX;
}
}
TiedCinWitnessData Build(const tl::fea::ShellBatchBinding& binding,const std::vector<ReferenceRow>& parents,
        native_search::ClassificationView<native_search::CinAttachmentRow> attachments,
        const tl::fea::NodalNodeDomain& domain,TiedCinWitnessLimits limits) {
    using output::Require;
    Require(binding.prepared() && domain.prepared() && attachments.data && attachments.count &&
        parents.size()==binding.qeph_count()+binding.t3_count()+binding.qbat_count(),
        "CIN witness preparation needs the complete shell family/source scope");
    std::uint64_t previous=0;
    for (const auto& node:binding.active_nodes()) {
        Require(node.source_id>previous,"CIN source binding node order is not ascending NID");
        previous=node.source_id;
    }
    std::vector<std::size_t> offsets(binding.node_count()+1),cursor(binding.node_count());
    for (const auto& row:parents) {
        const auto parent=Read(binding,row);
        for (unsigned slot=0;slot<parent.arity;++slot) ++offsets[parent.nodes[slot]+1];
    }
    for (std::size_t n=1;n<offsets.size();++n) offsets[n]+=offsets[n-1];
    std::copy(offsets.begin(),offsets.end()-1,cursor.begin());
    std::vector<std::uint32_t> incidence(offsets.back());
    Require(offsets.capacity()<=binding.node_count()+1 && cursor.capacity()<=binding.node_count() &&
        incidence.capacity()<=4*parents.size(),"Actual CIN incidence capacities exceed preflight");
    for (std::size_t row=0;row<parents.size();++row) {
        const auto parent=Read(binding,parents[row]);
        for (unsigned slot=0;slot<parent.arity;++slot)
            incidence[cursor[parent.nodes[slot]]++]=static_cast<std::uint32_t>(row);
    }
    TiedCinWitnessData next;
    next.ranges.resize(attachments.count);
    // One bounded reservation, no vector growth or per-parent allocation.
    next.witnesses.reserve(limits.witnesses);
    next.origins.reserve(limits.witnesses);
    next.counts.source_parents=parents.size();
    next.counts.attachments=attachments.count;
    for (std::size_t row=0;row<attachments.count;++row) {
        const auto& patch=attachments.data[row];
        std::array<std::size_t,4> nodes;
        for (unsigned slot=0;slot<4;++slot) {
            Require(patch.master_domain_nodes[slot]<domain.node_count(),"Invalid CIN master domain index");
            nodes[slot]=Find(binding,domain.nodes()[patch.master_domain_nodes[slot]].source_id);
            Require(nodes[slot]!=SIZE_MAX,"CIN master has no retained shell source incidence");
        }
        auto& range=next.ranges[row];
        range.offset=static_cast<std::uint32_t>(next.witnesses.size());
        for (auto at=offsets[nodes[0]];at<offsets[nodes[0]+1];++at) {
            const auto source_row=incidence[at];
            const auto& original=parents[source_row];
            const auto parent=Read(binding,original);
            if (!Contains(parent,nodes)) continue;
            Require(original.role==modelio::vehicle::SourceShellRole::ConstitutiveShell,
                    "CIN positive-shell profile cannot consume an original rigid-part witness");
            Require(next.witnesses.size()<limits.witnesses,"CIN positive-witness roster exceeds declared count cap");
            cin_stage::ActiveWitness witness;
            witness.source_element_id=original.element_id;
            witness.native_parent_index=static_cast<std::uint32_t>(original.reference_index);
            witness.family=parent.arity==3 ? cin_stage::WitnessFamily::ShellTriangle : cin_stage::WitnessFamily::ShellQuad;
            TiedCinWitnessOrigin origin;
            origin.source_part_id=original.part_id;
            origin.source_parent_row=source_row;
            origin.canonical_parent=original.canonical_parent;
            origin.family=parent.family;
            for (unsigned slot=0;slot<4;++slot) {
                const auto& source_node=binding.nodes()[parent.nodes[slot]];
                origin.source_node_ids[slot]=source_node.source_id;
                const auto mapped=domain.Find(source_node.source_id);
                witness.nodes[slot]=mapped==SIZE_MAX ? UINT32_MAX : static_cast<std::uint32_t>(mapped);
                if (mapped==SIZE_MAX) ++next.counts.missing_domain_slots;
                else Require(tl::fea::nodal_domain_detail::SamePosition(source_node.position,domain.nodes()[mapped].position),
                    "CIN witness source/domain coordinate bits disagree");
            }
            const bool declared=original.element_id==patch.master_source.element_id;
            if (declared) {
                Require(original.part_id==patch.master_source.part_id,"Declared CIN master PID differs from source witness");
                ++next.counts.declared_parent_witnesses;
            } else ++next.counts.additional_containing_parents;
            next.witnesses.push_back(witness);
            next.origins.push_back(origin);
        }
        range.count=static_cast<std::uint32_t>(next.witnesses.size()-range.offset);
        next.counts.maximum_per_row=std::max(next.counts.maximum_per_row,std::size_t(range.count));
        if (!range.count) ++next.counts.rows_without_shell_witness;
    }
    next.counts.witnesses=next.witnesses.size();
    return next;
}
}
