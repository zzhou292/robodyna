#include "Fixture.h"
#include <cmath>
namespace crash::cases::vehicle_startup::cin_witness_test {
TEST(TiedCinWitnessValues, SourceOrderedCoincidentLayersAndTriangleContainmentStayDistinct) {
    Fixture f;
    const auto result=f.Build();
    ASSERT_EQ(result.ranges.size(),2u);
    EXPECT_EQ(result.ranges[0].count,3u);
    EXPECT_EQ(result.ranges[1].count,4u);
    std::vector<std::uint64_t> ids;
    for (const auto& witness:result.witnesses) ids.push_back(witness.source_element_id);
    EXPECT_EQ(ids,(std::vector<std::uint64_t>{103,101,100,103,101,102,100}));
    EXPECT_EQ(result.counts.maximum_per_row,4u);
    EXPECT_EQ(result.counts.declared_parent_witnesses,2u);
    EXPECT_EQ(result.counts.additional_containing_parents,5u);
    EXPECT_EQ(result.counts.missing_domain_slots,0u);
    EXPECT_EQ(result.origins[0].family,fe::ShellBindingFamily::Qbat);
    EXPECT_EQ(result.origins[1].family,fe::ShellBindingFamily::Qeph);
    const auto& triangle=result.witnesses[5];
    EXPECT_EQ(triangle.family,cin_stage::WitnessFamily::ShellTriangle);
    EXPECT_EQ(triangle.nodes[2],triangle.nodes[3]);
    for (std::size_t i=0;i<result.witnesses.size();++i) {
        const auto& origin=result.origins[i];
        EXPECT_EQ(origin.canonical_parent,f.parents[origin.source_parent_row].canonical_parent);
        EXPECT_EQ(origin.source_part_id,f.parents[origin.source_parent_row].part_id);
        for (unsigned slot=0;slot<4;++slot)
            EXPECT_EQ(f.domain.nodes()[result.witnesses[i].nodes[slot]].source_id,origin.source_node_ids[slot]);
    }
}
TEST(TiedCinWitnessValues, MissingExtraQuadNodeRemainsRecordedAndExactSourceBitsAreChecked) {
    Fixture f;
    const auto original=f.Build();
    // Keep only the triangle patch; the fourth Q4 witness node is deliberately
    // outside this smaller geometric domain. All four source witnesses remain.
    std::vector<fe::NodalDomainNode> nodes;
    for (const auto& node:f.domain.nodes()) if (node.source_id!=13) nodes.push_back(node);
    fe::NodalNodeDomain small;
    ASSERT_TRUE(small.Initialize({91,nodes.data(),nodes.size()}));
    auto patch=f.attachments.back();
    for (unsigned slot=0;slot<4;++slot) patch.master_domain_nodes[slot]=small.Find(slot==3 ? 12 : 10+slot);
    const auto pending=cin_witness_detail::Build(f.binding,f.parents,{&patch,1},small,{});
    EXPECT_EQ(pending.counts.witnesses,4u);
    EXPECT_EQ(pending.counts.missing_domain_slots,3u);
    EXPECT_EQ(pending.origins.front().source_node_ids[3],13u);
    EXPECT_EQ(pending.witnesses.front().nodes[3],UINT32_MAX);
    nodes.back().position.z=-0.;
    fe::NodalNodeDomain changed;
    ASSERT_TRUE(changed.Initialize({91,nodes.data(),nodes.size()}));
    EXPECT_THROW(cin_witness_detail::Build(f.binding,f.parents,{&patch,1},changed,{}),std::exception);
    EXPECT_EQ(f.Build().counts.witnesses,original.counts.witnesses);
}
TEST(TiedCinWitnessValues, LateIdentityRoleAndWitnessCapRejectWithoutReplacingPriorOutput) {
    Fixture f;
    auto saved=f.Build();
    const auto before=saved.witnesses.back().source_element_id;
    ++f.parents.back().element_id;
    EXPECT_THROW(saved=f.Build(),std::exception);
    EXPECT_EQ(saved.witnesses.back().source_element_id,before);
    --f.parents.back().element_id;
    f.parents.back().role=modelio::vehicle::SourceShellRole::OriginalRigidPart;
    EXPECT_THROW(saved=f.Build(),std::exception);
    f.parents.back().role=modelio::vehicle::SourceShellRole::ConstitutiveShell;
    TiedCinWitnessLimits limits;
    limits.witnesses=6;
    EXPECT_THROW(saved=f.Build(limits),std::exception);
    EXPECT_EQ(saved.witnesses.back().source_element_id,before);
    limits.witnesses=7;
    EXPECT_EQ(f.Build(limits).counts.witnesses,7u);
}
TEST(TiedCinWitnessValues, ExactBytePreflightAndActualCapacityHaveNoImplicitGrowth) {
    Fixture f;
    TiedCinWitnessLimits limits;
    limits.witnesses=7;
    const auto forecast=cin_witness_detail::Budget(10000,4,4,2,1000,limits);
    EXPECT_EQ(forecast.incidence_scratch_bytes,9*sizeof(std::size_t)+16*sizeof(std::uint32_t));
    limits.host_bytes=forecast.total_host_bytes-1;
    EXPECT_THROW(cin_witness_detail::Budget(10000,4,4,2,1000,limits),std::exception);
    ++limits.host_bytes;
    EXPECT_EQ(cin_witness_detail::Budget(10000,4,4,2,1000,limits).total_host_bytes,limits.host_bytes);
    auto result=f.Build(limits);
    EXPECT_NO_THROW(cin_witness_detail::CheckActualCapacity(result,forecast,limits));
    result.witnesses.reserve(14);
    EXPECT_THROW(cin_witness_detail::CheckActualCapacity(result,forecast,limits),std::exception);
    EXPECT_THROW(cin_witness_detail::Budget(SIZE_MAX,4,4,2,1000,{}),std::exception);
}
}
