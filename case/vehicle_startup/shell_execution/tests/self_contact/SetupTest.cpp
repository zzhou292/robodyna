#include "Source.h"
#include <cstring>
#include <iostream>

namespace crash::cases::vehicle_startup::shell_execution::self_contact_test {
namespace app_contact = crash::cases::vehicle_self_contact;

namespace {

tlfea::contact::SelfContactActiveUseSource SupportSource() {
    const auto& attachments = PhysicalAttachments();
    const auto& roster = attachments.witnesses().data();
    return {&Execution().model().rigid_assembly(),
        {&attachments.attachments().model(), roster.ranges.data(),
         roster.witnesses.data(), roster.ranges.size(),
         roster.witnesses.size()}};
}

void CheckSupportPartition(const app_contact::SupportRoleCounts& counts) {
    EXPECT_EQ(counts.total, counts.ordinary + counts.rigid +
        counts.cin_master + counts.cin_secondary);
    EXPECT_EQ(counts.rigid,
        counts.complete_rigid + counts.partial_or_mixed_rigid);
}

}  // namespace

TEST(VehicleSelfContactSource,
     AppOwnedOriginalSelectionActiveUsesAndAreasAtEveryFixedLevel) {
    const auto& execution = Execution();
    const auto& attachments = PhysicalAttachments();
    const auto& original = OriginalContactSelection();
    const auto& fields = original.data().source_fields;
    ASSERT_EQ(fields.slave_set_id, 1000002u);
    ASSERT_EQ(fields.master_set_id, 5000002u);
    ASSERT_EQ(fields.static_friction, .2);
    ASSERT_EQ(fields.dynamic_friction, .1);
    ASSERT_EQ(fields.decay_coefficient, .001);
    ASSERT_EQ(fields.soft, 1);
    const std::size_t expected_facets[3]{
        653055, 2612220, 10448880};
    const std::size_t expected_vertex_uses[3]{
        1327239, 2970441, 8216010};
    const std::size_t expected_edge_uses[3]{
        1643202, 5245569, 18327798};

    for (unsigned level = 0; level <= 2; ++level) {
        const auto setup = app_contact::VehicleSelfContactSetup::Prepare(
            execution, attachments, original, {level});
        ASSERT_TRUE(setup.MatchesSource(
            execution, attachments, original));
        ASSERT_TRUE(setup.identity());
        EXPECT_EQ(&setup.execution().physical(), &execution.physical());
        EXPECT_EQ(&setup.attachments().witnesses(),
            &attachments.witnesses());
        EXPECT_EQ(&setup.original().data(), &original.data());
        EXPECT_TRUE(setup.surface().MatchesPhysical(execution.physical()));
        EXPECT_EQ(setup.config().facet_level, level);

        const auto& source = setup.counts();
        EXPECT_EQ(source.original_selected_parts, 861u);
        EXPECT_EQ(source.original_shell_parts, 850u);
        EXPECT_EQ(source.original_retained_shell_parts, 842u);
        EXPECT_EQ(source.selected_shell_parts, 842u);
        EXPECT_EQ(source.selected_offset_parts, 0u);
        EXPECT_EQ(source.omitted_tire_parts, 8u);
        EXPECT_EQ(source.unsupported_non_shell_parts, 11u);
        EXPECT_EQ(source.original_shells, 345904u);
        EXPECT_EQ(source.selected_shell_parents, 337092u);
        EXPECT_EQ(source.selected_offset_parents, 0u);
        EXPECT_EQ(source.omitted_tire_shells, 8812u);
        EXPECT_EQ(source.unsupported_solids, 2952u);
        EXPECT_EQ(source.unsupported_beams, 0u);
        EXPECT_EQ(source.q4_parents, 315963u);
        EXPECT_EQ(source.t3_parents, 21129u);
        EXPECT_EQ(setup.inventory().selected_part_ids.size(), 842u);
        EXPECT_EQ(
            setup.inventory().retained_shell_part_ids.size(), 842u);
        EXPECT_EQ(setup.inventory().offset_part_ids.size(), 0u);
        EXPECT_EQ(setup.inventory().offset_parents.size(), 0u);

        const auto& topology = setup.census().topology;
        EXPECT_EQ(topology.parents, 337092u);
        EXPECT_EQ(topology.q4_parents, 315963u);
        EXPECT_EQ(topology.t3_parents, 21129u);
        EXPECT_EQ(topology.facets, expected_facets[level]);
        EXPECT_EQ(topology.parent_local_vertex_uses,
            expected_vertex_uses[level]);
        EXPECT_EQ(topology.parent_local_edge_uses,
            expected_edge_uses[level]);
        tlfea::contact::SelfContactActiveUseCounts expected;
        ASSERT_TRUE(tlfea::contact::CountSelfContactActiveUses(
            {315963, 21129, setup.surface().vertices().size(),
             setup.surface().edges().size(), level}, &expected));
        EXPECT_EQ(topology.canonical_vertices, expected.vertices);
        EXPECT_EQ(topology.canonical_edges, expected.edges);
        EXPECT_EQ(topology.parent_local_vertex_uses,
            expected.vertex_uses);
        EXPECT_EQ(topology.parent_local_edge_uses,
            expected.edge_uses);

        const auto& active = setup.active_uses();
        const auto* rigid = active.rigid();
        ASSERT_NE(rigid, nullptr);
        EXPECT_EQ(rigid->groups().data(),
            execution.model().rigid_assembly().groups().data());
        EXPECT_EQ(rigid->members().data(),
            execution.model().rigid_assembly().members().data());
        const auto cin = active.cin();
        const auto& roster = attachments.witnesses().data();
        ASSERT_NE(cin.model, nullptr);
        EXPECT_EQ(cin.model->rows().data,
            attachments.attachments().model().rows().data);
        ASSERT_EQ(cin.range_count, roster.ranges.size());
        ASSERT_EQ(cin.witness_count, roster.witnesses.size());
        EXPECT_EQ(cin.range_count, 11165u);
        EXPECT_EQ(cin.witness_count, 13173u);
        EXPECT_EQ(std::memcmp(cin.ranges, roster.ranges.data(),
            cin.range_count * sizeof(*cin.ranges)), 0);
        EXPECT_EQ(std::memcmp(cin.witnesses, roster.witnesses.data(),
            cin.witness_count * sizeof(*cin.witnesses)), 0);

        const auto& support = setup.census().support;
        CheckSupportPartition(support.vertex_uses);
        CheckSupportPartition(support.edge_endpoints);
        CheckSupportPartition(support.all_weighted_supports);
        EXPECT_EQ(support.vertex_uses.total,
            expected_vertex_uses[level]);
        EXPECT_EQ(support.edge_endpoints.total,
            2 * expected_edge_uses[level]);
        EXPECT_TRUE(support.complete_static_cin_roster);
        EXPECT_TRUE(support.runtime_activity_and_release_pending);
        EXPECT_EQ(support.admitted_runtime_tied_exclusions, 0u);
        EXPECT_GT(
            setup.census().reference_area.q4_parent_area_m2.lower, 0);
        EXPECT_GT(
            setup.census().reference_area.t3_parent_area_m2.lower, 0);
        EXPECT_GT(
            setup.census().reference_area.total_parent_area_m2.lower, 0);
        EXPECT_GT(
            setup.census().reference_area.directed_vertex_area_m2.lower, 0);

        std::cout << "Selected self-contact setup level=" << level
                  << " parents=" << topology.parents
                  << " facets=" << topology.facets
                  << " canonical_vertices=" << topology.canonical_vertices
                  << " canonical_edges=" << topology.canonical_edges
                  << " vertex_uses=" << topology.parent_local_vertex_uses
                  << " edge_uses=" << topology.parent_local_edge_uses
                  << " ordinary="
                  << support.all_weighted_supports.ordinary
                  << " rigid=" << support.all_weighted_supports.rigid
                  << " cin_master="
                  << support.all_weighted_supports.cin_master
                  << " cin_secondary="
                  << support.all_weighted_supports.cin_secondary
                  << " setup_peak=" << setup.forecast().peak_host_bytes
                  << '\n';
        RecordProperty("level_" + std::to_string(level) + "_facets",
            std::to_string(topology.facets));
        RecordProperty("level_" + std::to_string(level) +
                "_canonical_vertices",
            std::to_string(topology.canonical_vertices));
        RecordProperty("level_" + std::to_string(level) +
                "_canonical_edges",
            std::to_string(topology.canonical_edges));
        RecordProperty("level_" + std::to_string(level) +
                "_weighted_supports",
            std::to_string(support.all_weighted_supports.total));
        RecordProperty("level_" + std::to_string(level) + "_ordinary",
            std::to_string(support.all_weighted_supports.ordinary));
        RecordProperty("level_" + std::to_string(level) + "_rigid",
            std::to_string(support.all_weighted_supports.rigid));
        RecordProperty("level_" + std::to_string(level) + "_cin_master",
            std::to_string(support.all_weighted_supports.cin_master));
        RecordProperty("level_" + std::to_string(level) + "_cin_secondary",
            std::to_string(support.all_weighted_supports.cin_secondary));
    }
}

TEST(VehicleSelfContactSource,
     SetupCapLateIdentityAndRetryPreserveExistingImmutableSource) {
    const auto& execution = Execution();
    const auto& attachments = PhysicalAttachments();
    const auto& original = OriginalContactSelection();
    const app_contact::Config config{0};
    const auto forecast = app_contact::VehicleSelfContactSetup::Preflight(
        execution, attachments, original, config);
    app_contact::SetupLimits exact;
    exact.host_bytes = forecast.peak_host_bytes;
    EXPECT_EQ(app_contact::VehicleSelfContactSetup::Preflight(
        execution, attachments, original, config, exact).peak_host_bytes,
        exact.host_bytes);
    auto setup = app_contact::VehicleSelfContactSetup::Prepare(
        execution, attachments, original, config, exact);
    const auto identity = setup.identity();
    const auto* first = &setup.surface().parents()[0];
    auto short_limit = exact;
    --short_limit.host_bytes;
    EXPECT_THROW(app_contact::VehicleSelfContactSetup::Preflight(
        execution, attachments, original, config, short_limit),
        std::runtime_error);
    EXPECT_TRUE(identity.Matches(setup.identity()));
    EXPECT_EQ(&setup.surface().parents()[0], first);

    auto altered = original.data();
    for (auto i = altered.parts.size(); i-- > 0;) {
        if (!altered.parts[i].retained_shell_part) continue;
        ++altered.parts[i].shells;
        ++altered.counts.shells;
        ++altered.counts.retained_shells;
        break;
    }
    EXPECT_THROW(app_contact::SelectedSelfContactSource::Preflight(
        execution.physical(), altered, SupportSource(), config),
        std::runtime_error);
    EXPECT_TRUE(identity.Matches(setup.identity()));
    EXPECT_EQ(&setup.surface().parents()[0], first);

    auto duplicate = original.data();
    duplicate.selected_part_ids.back() =
        duplicate.selected_part_ids.front();
    duplicate.parts.back().part_id =
        duplicate.selected_part_ids.back();
    EXPECT_THROW(app_contact::SelectedSelfContactSource::Preflight(
        execution.physical(), duplicate, SupportSource(), config),
        std::runtime_error);
    EXPECT_TRUE(setup.MatchesSource(execution, attachments, original));

    const auto retried = app_contact::VehicleSelfContactSetup::Prepare(
        execution, attachments, original, config, exact);
    EXPECT_EQ(retried.forecast().peak_host_bytes, exact.host_bytes);
    EXPECT_FALSE(identity.Matches(retried.identity()));
    EXPECT_TRUE(retried.MatchesSource(
        execution, attachments, original));
}

}  // namespace crash::cases::vehicle_startup::shell_execution::self_contact_test
