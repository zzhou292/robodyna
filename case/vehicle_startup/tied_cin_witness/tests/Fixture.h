#pragma once
#include "../Internal.h"
#include "lib_utest/qualification/qbat_binding/Fixture.h"
namespace crash::cases::vehicle_startup::cin_witness_test {
namespace fe=tl::fea;
struct Fixture {
    qbat_binding_test::Fixture geometry;
    fe::ShellBatchBinding binding;
    fe::NodalNodeDomain domain;
    std::vector<ReferenceRow> parents;
    std::vector<native_search::CinAttachmentRow> attachments;
    Fixture() {
        geometry.t.nodes={0,1,2};
        for (unsigned i=0;i<3;++i) {
            geometry.t.reference.position[i]=geometry.q[0].reference.position[i];
            geometry.t.reference.node_ids[i]=10+i;
        }
        auto input=geometry.Input();
        input.shells.node_count=4;
        output::Require(binding.InitializeFormulations(input).status==fe::ShellBindingStatus::Success,
                        "Tiny shared-layer native binding rejected");
        for (const auto pair:std::vector<std::pair<ReferenceFamily,std::size_t>>{
            {ReferenceFamily::Qbat,0},{ReferenceFamily::Qeph,1},{ReferenceFamily::T3,0},{ReferenceFamily::Qeph,0}}) {
            ReferenceRow row;
            row.family=pair.first;
            row.reference_index=pair.second;
            row.element_id=pair.first==ReferenceFamily::Qbat ? 103 : pair.first==ReferenceFamily::T3 ? 102 : 100+pair.second;
            row.part_id=row.element_id+1000;
            row.canonical_parent=7+parents.size();
            row.status=ReferenceStatus::Success;
            parents.push_back(row);
        }
        std::vector<fe::NodalDomainNode> nodes;
        for (std::size_t i=4;i-->0;) nodes.push_back({binding.nodes()[i].source_id,binding.nodes()[i].position});
        output::Require(bool(domain.Initialize({91,nodes.data(),nodes.size()})),"Tiny domain rejected");
        attachments.resize(2);
        for (std::size_t row=0;row<2;++row) {
            auto& patch=attachments[row];
            patch.original_nsv_row=row+3;
            patch.ordered_master_rank=row+8;
            patch.master_source.element_id=row ? 102 : 100;
            patch.master_source.part_id=patch.master_source.element_id+1000;
            for (unsigned slot=0;slot<4;++slot) patch.master_domain_nodes[slot]=domain.Find(10+(row && slot==3 ? 2 : slot));
            if (row) patch.topology=native_search::CinMasterTopology::TriangleRepeatedThird;
        }
    }
    TiedCinWitnessData Build(TiedCinWitnessLimits limits={}) const {
        return cin_witness_detail::Build(binding,parents,{attachments.data(),attachments.size()},domain,limits);
    }
};
}
