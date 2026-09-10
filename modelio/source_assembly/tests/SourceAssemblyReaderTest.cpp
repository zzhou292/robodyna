#include "AssemblyTestSupport.h"
#include <algorithm>
#include <set>

namespace crash::modelio::assembly::test {
TEST(SourceAssemblyReader,RetainsFrozenSchemaBytesAndCompleteSourceGeometry) {
    const auto model = Load(); const auto& data = model.data();
    EXPECT_EQ(data.schema, InventorySchema); EXPECT_EQ(data.identity.bytes, 1731843u);
    EXPECT_EQ(data.identity.sha256, PinnedYarisSixPartInventory().sha256);
    EXPECT_EQ(data.authenticated_bytes, FixtureBytes());
    ASSERT_EQ(data.nodes.size(), 1030u); ASSERT_EQ(data.parents.size(), 915u);
    EXPECT_EQ(data.qeph_count, 804u); EXPECT_EQ(data.t3_count, 111u);
    const auto probe = std::find_if(data.nodes.begin(), data.nodes.end(), [](const auto& node) { return node.source_id == 2121208; });
    ASSERT_NE(probe, data.nodes.end());
    EXPECT_EQ(output::Bits(probe->position_m.x), 0xbfd5cc8e993570a5ULL);
    EXPECT_EQ(output::Bits(probe->position_m.y), 0x3fe1141f9ca86cc7ULL);
    EXPECT_EQ(output::Bits(probe->position_m.z), 0x3fe090f3d8aa50a1ULL);
    const std::vector<SourceId> expected{2000119,2000120,2000145,2000157,2000165,2000260};
    ASSERT_EQ(data.parts.size(), expected.size());
    std::set<SourceId> ids;
    for (std::size_t i = 0; i < expected.size(); ++i) {
        EXPECT_EQ(data.parts[i].id, expected[i]); EXPECT_GT(data.parts[i].parent_count, 0u);
        EXPECT_EQ(output::Sha256(data.parts[i].source.raw_text), data.parts[i].source.sha256);
    }
    for (const auto& parent : data.parents) {
        EXPECT_TRUE(ids.insert(parent.source_id).second);
        EXPECT_EQ(parent.raw_record[0], parent.source_id); EXPECT_EQ(parent.raw_record[1], parent.part_id);
        for (unsigned n = 0; n < 4; ++n) {
            EXPECT_EQ(data.nodes[parent.nodes[n]].source_id, parent.raw_record[n + 2]);
            EXPECT_EQ(data.nodes[parent.nodes[n]].canonical_index, parent.canonical_nodes[n]);
        }
        if (parent.arity == 3) EXPECT_EQ(parent.raw_record[4], parent.raw_record[5]);
    }
}
TEST(SourceAssemblyReader,RetainsTwoCurvesSixTablesAndOriginalElforms) {
    const auto model = Load(); const auto& d = model.data();
    ASSERT_EQ(d.curves.size(), 2u); ASSERT_EQ(d.materials.size(), 6u); ASSERT_EQ(d.sections.size(), 6u);
    EXPECT_EQ(d.curves[0].id, 2100180u); EXPECT_EQ(d.curves[0].plastic_strain.size(), 17u);
    EXPECT_EQ(d.curves[1].id, 2100270u); EXPECT_EQ(d.curves[1].plastic_strain.size(), 46u);
    SameBits(d.curves[0].stress_pa.front(), 180e6); SameBits(d.curves[1].stress_pa.front(), 270e6);
    SameBits(d.curves[1].stress_pa[1], 290693298.29999995);
    EXPECT_EQ(d.sections[0].source_elform, 16u); EXPECT_EQ(d.sections[1].source_elform, 2u);
    EXPECT_EQ(d.materials[2].curve_id, 2100180u); EXPECT_EQ(d.materials[0].curve_id, 2100270u);
    for (const auto& material : d.materials) {
        EXPECT_EQ(material.cards.size(), 4u); EXPECT_EQ(material.cards[1].blank_mask, 232u);
        SameBits(material.rate_c_per_s, 8000); SameBits(material.rate_p, 8);
        EXPECT_EQ(material.source_rate_type, 0u); EXPECT_FALSE(material.supplied_etan_pa);
    }
    SameBits(d.units.mass_to_kg, 1000); SameBits(d.units.length_to_m, .001);
}
TEST(SourceAssemblyReader,InternalGroupsAndReleasedFrontierRemainSeparateDeclarations) {
    const auto model = Load(); const auto& d = model.data();
    std::set<SourceId> internal;
    std::size_t groups = 0;
    for (const auto& group : d.nodal_rigid_groups) {
        EXPECT_EQ(group.rigid_cards[0].blank_mask, 250u);
        EXPECT_EQ(output::Sha256(group.rigid_source.raw_text), group.rigid_source.sha256);
        if (!group.internal) continue;
        EXPECT_EQ(group.id, 2200909u + groups++); EXPECT_TRUE(group.external_nodes.empty());
        ASSERT_EQ(group.members.size(), group.selected_global_nodes.size());
        for (std::size_t i = 0; i < group.members.size(); ++i) {
            EXPECT_TRUE(internal.insert(group.members[i]).second);
            EXPECT_EQ(d.nodes[group.selected_global_nodes[i]].source_id, group.members[i]);
        }
    }
    EXPECT_EQ(groups, 6u); EXPECT_EQ(internal.size(), 76u);
    EXPECT_EQ(d.boundary.nodal_rigid_ids, (std::vector<SourceId>{2200054,2200055,2200921,2200946}));
    EXPECT_EQ(d.boundary.spotweld_ids.size(), 13u); EXPECT_EQ(d.released_spotwelds.size(), 13u);
    EXPECT_EQ(d.boundary.external_node_ids.size(), 37u); EXPECT_EQ(d.boundary.external_part_ids.size(), 6u);
    ASSERT_EQ(d.boundary.tied_scopes.size(), 1u); EXPECT_EQ(d.boundary.tied_scopes[0].source_line, 36u);
    EXPECT_TRUE(d.boundary.tied_scopes[0].selected_slave_parts.empty());
    EXPECT_EQ(d.boundary.policy, "released_external_connections");
}
TEST(SourceAssemblyReader,CopiesShareImmutableEvidenceAndSurviveSourceMove) {
    auto original = Load(); auto copy = original;
    const auto* nodes = copy.data().nodes.data(); auto moved = std::move(original);
    EXPECT_EQ(moved.data().nodes.data(), nodes); EXPECT_EQ(copy.data().nodes.data(), nodes);
    EXPECT_EQ(copy.data().identity.sha256, PinnedYarisSixPartInventory().sha256);
    EXPECT_THROW(original.data(), std::runtime_error);
}
}  // namespace crash::modelio::assembly::test
