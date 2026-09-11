#include "TinyFixture.h"
#include <algorithm>

namespace crash::modelio::tied_shell::test {
namespace {
void Reject(const TinyFixture& fixture, const char* reason, Limits limits = {}) {
    try {
        fixture.Prepare(limits);
        FAIL() << "Malformed declaration was accepted";
    } catch (const std::exception& error) {
        EXPECT_NE(std::string(error.what()).find(reason), std::string::npos) << error.what();
    }
}
void SameEvidence(const Data& a, const Data& b) {
    ASSERT_EQ(a.sources.size(), b.sources.size());
    for (std::size_t i = 0; i < a.sources.size(); ++i) {
        EXPECT_EQ(a.sources[i].block.raw_text, b.sources[i].block.raw_text);
        EXPECT_EQ(a.sources[i].block.sha256, b.sources[i].block.sha256);
        EXPECT_EQ(a.sources[i].cards, b.sources[i].cards);
    }
    ASSERT_EQ(a.slave_nodes.size(), b.slave_nodes.size());
    for (std::size_t i = 0; i < a.slave_nodes.size(); ++i) {
        EXPECT_EQ(a.slave_nodes[i].id, b.slave_nodes[i].id);
        EXPECT_EQ(a.slave_nodes[i].canonical_index, b.slave_nodes[i].canonical_index);
        EXPECT_EQ(a.slave_nodes[i].source_codes, b.slave_nodes[i].source_codes);
    }
}
}
TEST(TiedDeclarationValues, CompleteIncidencePreservesSourceSlotsAndUnresolvedEvidence) {
    TinyFixture fixture;
    const auto d = fixture.Prepare();
    EXPECT_EQ(d.slave_set_id, 2u);
    EXPECT_EQ(d.master_set_id, 1u);
    EXPECT_EQ(d.counts.masters, 2u);
    EXPECT_EQ(d.counts.q4, 1u);
    EXPECT_EQ(d.counts.t3, 1u);
    EXPECT_EQ(d.counts.master_nodes, 5u);
    EXPECT_EQ(d.counts.slave_beams, 1u);
    EXPECT_EQ(d.counts.slave_solids, 1u);
    EXPECT_EQ(d.counts.slave_shells, 0u);
    EXPECT_EQ(d.counts.slave_nodes, 8u);
    EXPECT_EQ(d.counts.incidences, 10u);
    EXPECT_EQ(d.counts.shared_nodes, 1u);
    ASSERT_EQ(d.masters.size(), 2u);
    EXPECT_EQ(d.masters[1].id, 5011u);
    EXPECT_EQ(d.masters[1].arity, 3u);
    EXPECT_EQ(d.masters[1].canonical_index, 1u);
    ASSERT_EQ(d.slaves.size(), 2u);
    EXPECT_EQ(d.slaves[0].family, ElementFamily::Beam);
    EXPECT_EQ(d.slaves[1].family, ElementFamily::Solid);
    EXPECT_EQ(d.incidences[0].canonical_node, 5u);
    EXPECT_EQ(d.incidences[1].canonical_node, 6u); // N3 orientation is not a beam endpoint.
    EXPECT_EQ(d.incidences[2].canonical_node, 7u);
    EXPECT_EQ(d.incidences.back().canonical_node, 0u);
    EXPECT_EQ(d.incidences.back().local_slot, 7u);
    std::vector<SourceId> ids;
    for (const auto& node : d.slave_nodes) ids.push_back(node.id);
    EXPECT_EQ(ids, (std::vector<SourceId>{30,40,50,60,90,100,110,120}));
    EXPECT_EQ(d.master_nodes, (std::vector<std::uint32_t>{1,3,4,2,0}));
    EXPECT_EQ(d.counts.groups, 2u);
    EXPECT_EQ(d.counts.groups_touching_slaves, 2u);
    EXPECT_EQ(d.counts.groups_touching_masters, 1u);
    EXPECT_EQ(d.groups[0].source_nodes, (std::vector<SourceId>{90,10}));
    EXPECT_EQ(d.groups[0].slave_nodes, (std::vector<SourceId>{90}));
    EXPECT_EQ(d.groups[1].slave_nodes, (std::vector<SourceId>{30,60}));
    bool auxiliary = false, joint = false;
    for (const auto& row : d.unresolved_constraints) {
        if (row.filename == "auxiliary.key") {
            auxiliary = true;
            EXPECT_EQ(row.retained_source, SIZE_MAX);
            EXPECT_EQ(row.sha256, std::string(64, 'a'));
        }
        if (row.keyword == "*CONSTRAINED_JOINT_SPHERICAL_ID") {
            joint = true;
            ASSERT_LT(row.retained_source, d.sources.size());
            EXPECT_EQ(d.sources[row.retained_source].block.sha256, row.sha256);
        }
    }
    EXPECT_TRUE(auxiliary);
    EXPECT_TRUE(joint);
    EXPECT_EQ(d.ordering, NativeReadiness::Unresolved);
    EXPECT_EQ(d.classification, NativeReadiness::Unresolved);
    EXPECT_EQ(d.search, NativeReadiness::Unresolved);
    EXPECT_GT(d.owned_payload_bytes, sizeof(Data));
    EXPECT_LT(d.owned_payload_bytes, d.startup_budget_bytes);
}
TEST(TiedDeclarationValues, NonblankSetOptionsRemainRetainedAndUnresolved) {
    TinyFixture fixture(true);
    const auto d = fixture.Prepare();
    EXPECT_EQ(d.counts.slave_nodes, 8u);
    const auto& set = d.sources.at(d.slave_set_source);
    EXPECT_EQ(detail::CardId(set.cards.at(1).second, 1), 1u);
    const auto& nodes = d.sources.at(d.groups.at(0).node_set_source);
    EXPECT_EQ(detail::CardId(nodes.cards.at(0).second, 1), 1u);
    EXPECT_EQ(d.groups.at(0).source_nodes, (std::vector<SourceId>{90,10}));
    EXPECT_EQ(d.classification, NativeReadiness::Unresolved);
}
TEST(TiedDeclarationValues, CardValuesAndCompleteGroupCoverageAreAuthenticated) {
    TinyFixture changed;
    changed.AlterScope([](auto& d) {
        d["connections"]["nodal_rigid_groups"][1]["node_set_source"]["cards"][1]["text"].SetString("        30        61");
    });
    Reject(changed, "source card changed");
    changed = TinyFixture{};
    changed.AlterScope([](auto& d) { d["connections"]["nodal_rigid_groups"].PopBack(); });
    Reject(changed, "rigid-group declaration omitted");
    changed = TinyFixture{};
    changed.AlterScope([](auto& d) { d["connections"]["nodal_rigid_groups"][1]["source_node_ids"][1].SetUint64(61); });
    Reject(changed, "rigid membership differs");
    changed = TinyFixture{};
    changed.AlterScope([](auto& d) { d["declarations"]["parts"][3]["source_material_id"].SetUint64(2200); });
    Reject(changed, "material/section association changed");
}
TEST(TiedDeclarationValues, LateTopologyAndCoverageFailureLeavePriorValueAndAllowRetry) {
    TinyFixture fixture;
    auto visible = fixture.Prepare();
    const auto before = visible;
    auto altered = fixture;
    altered.AlterScope([](auto& d) { d["declarations"]["parts"][3]["counts"]["solids"].SetUint64(2); });
    EXPECT_THROW(visible = altered.Prepare(), std::exception);
    SameEvidence(visible, before);
    auto& array = altered.canonical.arrays[7];
    auto values = output::arrays::Decode<std::uint32_t>(array.descriptor, array.bytes);
    values.back() = 1; // Valid index, wrong original NID at the final incidence.
    array.bytes = output::arrays::Encode(array.descriptor.layout, values.data(), values.size());
    array.descriptor.sha256 = output::Sha256(array.bytes);
    Reject(altered, "connectivity differs");
    SameEvidence(visible, before);
    visible = fixture.Prepare();
    SameEvidence(visible, before);
}
TEST(TiedDeclarationValues, BudgetPrecedesBorrowedMemberAndCapsRemainExplicit) {
    TinyFixture fixture;
    const auto expected = detail::Preflight(fixture.canonical, {});
    auto limits = Limits{};
    limits.host_bytes = expected-1;
    auto malformed = fixture;
    malformed.member.back() = '!';
    Reject(malformed, "host byte cap", limits);
    Reject(malformed, "member authentication");
    limits.host_bytes = expected;
    EXPECT_EQ(fixture.Prepare(limits).startup_budget_bytes, expected);
    limits = {};
    limits.masters = 1;
    Reject(fixture, "master parent count", limits);
    limits = {};
    limits.slave_nodes = 7;
    Reject(fixture, "slave node count", limits);
    limits = {};
    limits.metadata_bytes = 64;
    Reject(fixture, "metadata byte cap", limits);
    limits = {};
    ++limits.groups;
    Reject(fixture, "Invalid tied declaration limits", limits);
    SameEvidence(fixture.Prepare(), fixture.Prepare());
}
} // namespace crash::modelio::tied_shell::test
