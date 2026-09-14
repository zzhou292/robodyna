#include "Source.h"
#include "case/vehicle_self_contact/VehicleSelfContactStartup.h"
#include "lib_src/collision/SelfContactCurrentRegularity.h"
#include <cfloat>
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

const app_contact::VehicleSelfContactSetup& LevelZeroSetup() {
    static const auto value = app_contact::VehicleSelfContactSetup::Prepare(
        Execution(), PhysicalAttachments(), OriginalContactSelection(), {0});
    return value;
}

void CheckSupportPartition(const app_contact::SupportRoleCounts& counts) {
    EXPECT_EQ(counts.total, counts.ordinary + counts.rigid +
        counts.cin_master + counts.cin_secondary);
    EXPECT_EQ(counts.rigid,
        counts.complete_rigid + counts.partial_or_mixed_rigid);
}

}  // namespace

TEST(VehicleSelfContactSource,
     LevelZeroMaterializationAndLevelOneTwoPreflightForecasts) {
    const auto& execution = Execution();
    const auto& attachments = PhysicalAttachments();
    const auto& original = OriginalContactSelection();
    const auto& fields = original.data().source_fields;
    ASSERT_EQ(fields.slave_set_id, 1000002u);
    ASSERT_EQ(fields.master_set_id, 0u);
    ASSERT_EQ(fields.static_friction, .2);
    ASSERT_EQ(fields.dynamic_friction, .1);
    ASSERT_EQ(fields.decay_coefficient, .001);
    ASSERT_EQ(fields.soft, 1);
    ASSERT_TRUE(fields.ignore_initial_penetration.has_value());
    EXPECT_EQ(*fields.ignore_initial_penetration, 1);
    const std::size_t expected_facets[3]{
        653055, 2612220, 10448880};
    const std::size_t expected_vertex_uses[3]{
        1327239, 2970441, 8216010};
    const std::size_t expected_edge_uses[3]{
        1643202, 5245569, 18327798};

    for (unsigned level = 0; level <= 2; ++level) {
        if (level) {
            const auto forecast =
                app_contact::VehicleSelfContactSetup::Preflight(
                    execution, attachments, original, {level});
            const auto& active = forecast.selected.active_uses;
            EXPECT_EQ(active.parents, 337092u);
            EXPECT_EQ(active.facets, expected_facets[level]);
            EXPECT_EQ(active.vertex_uses, expected_vertex_uses[level]);
            EXPECT_EQ(active.edge_uses, expected_edge_uses[level]);
            tlfea::contact::SelfContactActiveUseCounts expected;
            ASSERT_TRUE(tlfea::contact::CountSelfContactActiveUses(
                {315963, 21129, ContactSurface().vertices().size(),
                 ContactSurface().edges().size(), level}, &expected));
            EXPECT_EQ(active.vertices, expected.vertices);
            EXPECT_EQ(active.edges, expected.edges);
            std::cout << "Preflight-only selected self-contact level="
                      << level << " parents=" << active.parents
                      << " facets_forecast=" << active.facets
                      << " vertex_uses_forecast=" << active.vertex_uses
                      << " edge_uses_forecast=" << active.edge_uses
                      << " (active-use inventory not materialized)\n";
            RecordProperty("level_" + std::to_string(level) +
                    "_facets_forecast",
                std::to_string(active.facets));
            RecordProperty("level_" + std::to_string(level) +
                    "_canonical_vertices_forecast",
                std::to_string(active.vertices));
            RecordProperty("level_" + std::to_string(level) +
                    "_canonical_edges_forecast",
                std::to_string(active.edges));
            continue;
        }
        const auto& setup = LevelZeroSetup();
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
        CheckSupportPartition(
            support.vf_parent_local_vertex_use_support_occurrences);
        CheckSupportPartition(
            support.ee_stored_endpoint_support_occurrences);
        CheckSupportPartition(
            support.combined_stored_support_occurrences);
        EXPECT_EQ(
            support.vf_parent_local_vertex_use_support_occurrences.total,
            expected_vertex_uses[level]);
        EXPECT_EQ(support.ee_stored_endpoint_support_occurrences.total,
            2 * expected_edge_uses[level]);
        EXPECT_TRUE(support.complete_static_cin_roster);
        EXPECT_TRUE(support.runtime_activity_and_release_pending);
        EXPECT_TRUE(support.parent_activity_pending);
        EXPECT_EQ(support.parent_activity_pending_parents, 337092u);
        EXPECT_TRUE(support.same_parent_regularity_pending);
        EXPECT_EQ(support.same_parent_regularity_pending_edge_uses,
            expected_edge_uses[level]);
        EXPECT_TRUE(support.nonlocal_ee_force_area_pending);
        EXPECT_EQ(support.nonlocal_ee_force_area_pending_edge_uses,
            expected_edge_uses[level]);
        EXPECT_EQ(support.runtime_tied_exclusions, 0u);
        EXPECT_EQ(setup.census().runtime_coefficients
                      .applied_source_friction_fields,
            0u);
        EXPECT_EQ(setup.census().runtime_coefficients
                      .applied_source_damping_fields,
            0u);
        EXPECT_EQ(setup.census().runtime_coefficients
                      .applied_source_soft_fields,
            0u);
        const auto runtime_shape =
            app_contact::VehicleSelfContactStartup::SourceShape(setup);
        EXPECT_EQ(runtime_shape.facet_level, 0u);
        EXPECT_EQ(runtime_shape.physical_nodes, 372435u);
        EXPECT_EQ(runtime_shape.selected_parents, 337092u);
        EXPECT_EQ(runtime_shape.fixed_facets, 653055u);
        EXPECT_EQ(runtime_shape.applied_original_friction_fields, 0u);
        EXPECT_EQ(runtime_shape.applied_original_damping_fields, 0u);
        EXPECT_EQ(runtime_shape.applied_original_soft_fields, 0u);
        EXPECT_GT(
            setup.census().reference_area.q4_parent_area_m2.lower, 0);
        EXPECT_GT(
            setup.census().reference_area.t3_parent_area_m2.lower, 0);
        EXPECT_GT(
            setup.census().reference_area.total_parent_area_m2.lower, 0);
        EXPECT_GT(
            setup.census().reference_area.directed_vertex_area_m2.lower, 0);
        EXPECT_TRUE(
            setup.census().reference_area.directed_partition_certified);
        EXPECT_LE(setup.census().reference_area
                      .directed_partition_difference_m2.lower,
            0);
        EXPECT_GE(setup.census().reference_area
                      .directed_partition_difference_m2.upper,
            0);

        std::cout << "Materialized selected self-contact setup level="
                  << level
                  << " parents=" << topology.parents
                  << " facets=" << topology.facets
                  << " canonical_vertices=" << topology.canonical_vertices
                  << " canonical_edges=" << topology.canonical_edges
                  << " vertex_uses=" << topology.parent_local_vertex_uses
                  << " edge_uses=" << topology.parent_local_edge_uses
                  << " ordinary="
                  << support.combined_stored_support_occurrences.ordinary
                  << " rigid="
                  << support.combined_stored_support_occurrences.rigid
                  << " cin_master="
                  << support.combined_stored_support_occurrences.cin_master
                  << " cin_secondary="
                  << support.combined_stored_support_occurrences
                         .cin_secondary
                  << " setup_peak_reservation="
                  << setup.forecast().peak_host_reservation_bytes
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
                "_stored_support_occurrences",
            std::to_string(
                support.combined_stored_support_occurrences.total));
        RecordProperty("level_" + std::to_string(level) + "_ordinary",
            std::to_string(
                support.combined_stored_support_occurrences.ordinary));
        RecordProperty("level_" + std::to_string(level) + "_rigid",
            std::to_string(
                support.combined_stored_support_occurrences.rigid));
        RecordProperty("level_" + std::to_string(level) + "_cin_master",
            std::to_string(
                support.combined_stored_support_occurrences.cin_master));
        RecordProperty("level_" + std::to_string(level) + "_cin_secondary",
            std::to_string(
                support.combined_stored_support_occurrences.cin_secondary));
    }
}

TEST(VehicleSelfContactSource,
     InitialSelectedSourceHasCompleteCurrentRegularity) {
    const auto& setup = LevelZeroSetup();
    const auto* domain = setup.surface().physical()->domain();
    ASSERT_NE(domain, nullptr);
    std::vector<double> positions(3 * domain->node_count());
    for (std::size_t node = 0; node < domain->node_count(); ++node) {
        const auto point = domain->nodes()[node].position;
        positions[3 * node] = point.x;
        positions[3 * node + 1] = point.y;
        positions[3 * node + 2] = point.z;
    }
    std::vector<std::uint8_t> activity(
        setup.active_uses().parents().size(), 1);
    tlfea::contact::SelfContactCurrentRegularity regularity;
    const auto initialized = regularity.Initialize(
        setup.active_uses(),
        tlfea::contact::SelfContactCurrentRegularityLimits::Vehicle());
    ASSERT_EQ(initialized.status,
        tlfea::contact::SelfContactCurrentRegularityStatus::Ok)
        << initialized.message;
    tlfea::contact::SelfContactCurrentRegularityReceipt receipt;
    const tlfea::contact::SelfContactActivityView active{
        activity.data(), activity.data(), activity.size()};
    const auto report = regularity.Certify(
        {positions.data(), domain->node_count(), 3, 1},
        active, &receipt);
    ASSERT_EQ(report.status,
        tlfea::contact::SelfContactCurrentRegularityStatus::Ok)
        << report.message << " parent=" << report.parent
        << " facet=" << report.facet;
    ASSERT_TRUE(receipt.prepared());
    ASSERT_TRUE(receipt.MatchesInputs(
        {positions.data(), domain->node_count(), 3, 1}, active));
    const auto view = regularity.results();
    ASSERT_TRUE(view.complete);
    ASSERT_EQ(view.count, 337092u);
    EXPECT_EQ(view.summary.parents, 337092u);
    EXPECT_EQ(view.summary.facets, 653055u);
    EXPECT_EQ(view.summary.facets_evaluated, 653055u);
    EXPECT_EQ(view.summary.certified_parents, 337092u);
    EXPECT_EQ(view.summary.active_parents, 337092u);
    EXPECT_EQ(view.summary.removing_parents, 0u);
    EXPECT_EQ(view.summary.skipped_parents, 0u);
    EXPECT_GT(view.summary.minimum_scaled_jacobian_quality,
        64 * DBL_EPSILON);
    EXPECT_GT(
        view.summary.certified_current_area_enclosure_m2.lower, 0);
    std::cout << "Initial selected self-contact regularity parents="
              << view.summary.certified_parents
              << " facets=" << view.summary.facets_evaluated
              << " minimum_scaled_jacobian="
              << view.summary.minimum_scaled_jacobian_quality
              << " maximum_approximation_m="
              << view.summary.maximum_approximation_upper_m
              << '\n';
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
    exact.host_bytes = forecast.peak_host_reservation_bytes;
    EXPECT_EQ(app_contact::VehicleSelfContactSetup::Preflight(
        execution, attachments, original, config, exact)
                  .peak_host_reservation_bytes,
        exact.host_bytes);
    const auto& setup = LevelZeroSetup();
    ASSERT_EQ(setup.forecast().peak_host_reservation_bytes,
        exact.host_bytes);
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

    const auto retried = app_contact::VehicleSelfContactSetup::Preflight(
        execution, attachments, original, config, exact);
    EXPECT_EQ(retried.peak_host_reservation_bytes, exact.host_bytes);
    EXPECT_TRUE(identity.Matches(setup.identity()));
    EXPECT_TRUE(setup.MatchesSource(execution, attachments, original));
}

}  // namespace crash::cases::vehicle_startup::shell_execution::self_contact_test
