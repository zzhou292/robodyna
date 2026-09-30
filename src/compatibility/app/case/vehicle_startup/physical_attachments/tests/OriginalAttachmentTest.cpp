#include "Support.h"
#include <set>

namespace crash::cases::vehicle_startup::physical_attachments::test {
TEST(VehiclePhysicalAttachmentsOriginal, EveryOriginalPatchUsesActualCoefficientDomainAndSourceSupport) {
    const auto& value = Actual();
    const auto& physical = value.physical();
    const auto& domain = physical.source_domain().domain();
    const auto& cin = value.attachments().model();
    const auto& roster = value.witnesses().data();
    ASSERT_TRUE(cin.domain()->SharesStorage(domain));
    ASSERT_TRUE(physical.coefficients().domain()->SharesStorage(domain));
    ASSERT_EQ(cin.rows().count,11165u);
    EXPECT_EQ(roster.counts.witnesses,13173u);
    EXPECT_EQ(roster.counts.maximum_per_row,3u);
    EXPECT_EQ(roster.counts.missing_domain_slots,0u);
    EXPECT_EQ(roster.counts.rows_without_shell_witness,0u);
    std::set<std::uint64_t> master_nodes, secondary_nodes;
    std::size_t triangles = 0;
    for (std::size_t i = 0; i < cin.rows().count; ++i) {
        const auto& row = cin.rows().data[i];
        const auto& original = value.attachments().post_kinchk().classification().finalized();
        EXPECT_EQ(domain.nodes()[row.secondary_domain_node].source_id,original.secondary(i).id);
        EXPECT_EQ(row.master_source.element_id,original.selected_master(i).id);
        ASSERT_TRUE(secondary_nodes.insert(domain.nodes()[row.secondary_domain_node].source_id).second);
        EXPECT_EQ(physical.rigid_assembly().FindMember(row.secondary_domain_node),nullptr);
        for (auto node : row.master_domain_nodes) {
            ASSERT_LT(node,domain.node_count());
            EXPECT_EQ(physical.rigid_assembly().FindMember(node),nullptr);
            master_nodes.insert(domain.nodes()[node].source_id);
        }
        triangles += row.master_domain_nodes[2] == row.master_domain_nodes[3];
        const auto range = roster.ranges[i];
        ASSERT_GT(range.count,0u);
        for (std::size_t w = range.offset; w < range.offset + range.count; ++w) {
            const auto& witness = roster.witnesses[w];
            const auto& origin = roster.origins[w];
            for (unsigned slot = 0; slot < 4; ++slot) {
                ASSERT_LT(witness.nodes[slot],domain.node_count());
                EXPECT_EQ(domain.nodes()[witness.nodes[slot]].source_id,origin.source_node_ids[slot]);
            }
        }
    }
    EXPECT_EQ(master_nodes.size(),29585u);
    EXPECT_EQ(triangles,125u);
    EXPECT_EQ(cin.current_geometry(),native_search::CinAttachmentObligation::Pending);
    EXPECT_EQ(cin.master_activity_and_release(),native_search::CinAttachmentObligation::Pending);
    RecordProperty("physical_nodes",std::to_string(domain.node_count()));
    RecordProperty("original_cin_rows",std::to_string(cin.rows().count));
    RecordProperty("original_shell_witnesses",std::to_string(roster.counts.witnesses));
    RecordProperty("complete_forecast_bytes",std::to_string(value.forecast().total_bytes));
}
} // namespace crash::cases::vehicle_startup::physical_attachments::test
