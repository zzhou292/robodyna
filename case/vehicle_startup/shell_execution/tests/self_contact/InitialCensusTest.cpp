#include "Source.h"
#include "case/vehicle_self_contact/VehicleSelfContactInitialCensus.h"
#include <gtest/gtest.h>
#include <climits>
#include <iostream>

namespace crash::cases::vehicle_startup::shell_execution::self_contact_test {
namespace {

namespace app_contact = crash::cases::vehicle_self_contact;

TEST(VehicleSelfContactInitialCensus,
     ActualV5OwnerCountBeforeCapAndPrismRetentionCensus) {
    const auto& setup = LevelZeroSetup();
    auto dynamics =
        vehicle_dynamics::VehiclePhysicalDynamics::Prepare(
            Execution(), PhysicalAttachments(), {},
            &physical_model::supports_test::Joints());
    const auto initial = dynamics.accepted();
    app_contact::InitialCensusResult census;
    try {
        census =
            app_contact::VehicleSelfContactInitialCensus::Measure(
                setup, dynamics);
    } catch (const app_contact::InitialCensusCapacityError& error) {
        std::cout << "V5_INITIAL_SELF_CONTACT_CENSUS"
                  << " status=capacity_blocker"
                  << " required_parent_pairs="
                  << error.required_pairs()
                  << " reason=" << error.what() << '\n';
        throw;
    }

    EXPECT_EQ(census.source.nodes, initial.node_count);
    EXPECT_EQ(census.source.surface_parents, 337092u);
    EXPECT_EQ(census.source.active_parents, 337092u);
    EXPECT_EQ(census.source.physical_participants, 8u);
    EXPECT_EQ(census.source.cin_rows, 11165u);
    EXPECT_EQ(census.source.cin_witnesses, 13173u);
    EXPECT_TRUE(census.source.exact_setup_physical_identity);
    EXPECT_TRUE(census.source.exact_owner_stream_identity);
    EXPECT_TRUE(census.source.exact_rigid_identity);
    EXPECT_TRUE(census.source.exact_cin_identity);
    EXPECT_TRUE(census.source.exact_participant_source_identity);

    EXPECT_TRUE(census.count_before_cap_observed);
    EXPECT_GT(census.probe_required_pairs, 1u);
    EXPECT_LE(census.probe_required_pairs,
              static_cast<std::uint64_t>(INT_MAX));
    EXPECT_TRUE(
        census.cap_minus_one_reproduced_required_count);
    EXPECT_EQ(census.cap_minus_one_required_pairs,
              census.probe_required_pairs);
    EXPECT_TRUE(census.exact_capacity_succeeded);
    EXPECT_TRUE(census.complete_device_pair_keys);
    EXPECT_EQ(census.exact_pair_count,
              census.probe_required_pairs);
    EXPECT_EQ(census.forecast.exact.pair_capacity,
              census.exact_pair_count);
    EXPECT_EQ(census.forecast.cap_minus_one.pair_capacity + 1,
              census.forecast.exact.pair_capacity);
    EXPECT_LE(census.forecast.exact.device_bytes,
              std::size_t{2} << 30);
    EXPECT_LE(census.forecast.peak_census_host_reservation_bytes,
              std::size_t{2} << 30);
    EXPECT_EQ(census.forecast.fixed_workspace_host_bytes,
              census.forecast.surface_to_active_host_bytes +
              census.forecast.active_parent_host_bytes +
              census.forecast.pair_key_host_bytes +
              census.forecast.accepted_position_host_bytes +
              census.forecast.represented_triangle_host_bytes);

    const auto& capacity = census.capacity;
    EXPECT_EQ(capacity.current_inflated_aabb_overlap_parent_pairs,
              census.exact_pair_count);
    EXPECT_EQ(capacity.q4_q4_parent_pairs +
              capacity.q4_t3_parent_pairs +
              capacity.t3_t3_parent_pairs,
              census.exact_pair_count);
    EXPECT_GE(capacity.level0_facet_pairs,
              census.exact_pair_count);
    EXPECT_TRUE(capacity.sorted_unique_canonical_keys);
    EXPECT_TRUE(capacity.no_self_or_reversed_pair_keys);
    EXPECT_TRUE(capacity.descriptive_static_policy_only);
    EXPECT_FALSE(capacity.feature_discovery_performed);
    EXPECT_FALSE(capacity.intersection_processing_performed);
    EXPECT_FALSE(capacity.force_admission_performed);
    const auto& filters = census.filters;
    EXPECT_EQ(filters.represented_facet_pairs,
              capacity.level0_facet_pairs);
    EXPECT_EQ(filters.represented_facet_pairs, 5989248u);
    EXPECT_EQ(filters.excluded_same_rigid_group +
              filters.coordinate_aabb_separated +
              filters.face_axis_separated +
              filters.edge_cross_axis_separated +
              filters.exact_remaining,
              filters.represented_facet_pairs);
    EXPECT_EQ(filters.source_identity_hash,
              capacity.surface_active_source_hash);
    EXPECT_TRUE(filters.complete_disjoint_accounting);
    EXPECT_TRUE(filters.production_certificates_used);
    EXPECT_FALSE(filters.feature_discovery_performed);
    EXPECT_FALSE(filters.interval_crossing_performed);
    EXPECT_GT(filters.category_hash, 0u);
    EXPECT_GT(census.geometry_evaluation_us, 0u);
    EXPECT_GT(census.filter_census_us, 0u);
    EXPECT_TRUE(census.deterministic_rerun);
    EXPECT_EQ(census.rerun_pair_key_hash,
              capacity.pair_key_hash);
    EXPECT_TRUE(census.deterministic_filter_rerun);
    EXPECT_EQ(census.rerun_filter_hash,
              filters.category_hash);
    EXPECT_TRUE(census.accepted_owner_unchanged);
    EXPECT_EQ(dynamics.accepted().owner_id, initial.owner_id);
    EXPECT_EQ(dynamics.accepted().epoch, 0u);
    EXPECT_EQ(dynamics.accepted().time, 0);

    std::cout << "V5_INITIAL_SELF_CONTACT_CENSUS"
              << " status=complete"
              << " domain_source="
              << census.source.physical_domain_source_instance_id
              << " owner_id=" << census.source.owner_id
              << " configuration_id="
              << census.source.configuration_id
              << " qualification_id="
              << census.source.qualification_id
              << " nodes=" << census.source.nodes
              << " parents=" << census.source.surface_parents
              << " participants="
              << census.source.physical_participants
              << " rigid_groups=" << census.source.rigid_groups
              << " rigid_members=" << census.source.rigid_members
              << " cin_rows=" << census.source.cin_rows
              << " cin_witnesses=" << census.source.cin_witnesses
              << " required_parent_pairs="
              << census.probe_required_pairs
              << " probe_capacity="
              << census.forecast.count_probe.pair_capacity
              << " cap_minus_one_capacity="
              << census.forecast.cap_minus_one.pair_capacity
              << " cap_minus_one_required="
              << census.cap_minus_one_required_pairs
              << " exact_capacity="
              << census.forecast.exact.pair_capacity
              << " q4_q4=" << capacity.q4_q4_parent_pairs
              << " q4_t3=" << capacity.q4_t3_parent_pairs
              << " t3_t3=" << capacity.t3_t3_parent_pairs
              << " level0_facet_pairs="
              << capacity.level0_facet_pairs
              << " same_complete_rigid_group="
              << capacity.same_complete_rigid_group_parent_pairs
              << " shared_canonical_vertex="
              << capacity.shared_canonical_vertex_parent_pairs
              << " shared_canonical_edge="
              << capacity.shared_canonical_edge_parent_pairs
              << " pair_hash=" << capacity.pair_key_hash
              << " source_hash="
              << capacity.surface_active_source_hash
              << " represented_facet_pairs="
              << filters.represented_facet_pairs
              << " excluded_same_rigid_group="
              << filters.excluded_same_rigid_group
              << " coordinate_aabb_separated="
              << filters.coordinate_aabb_separated
              << " face_axis_separated="
              << filters.face_axis_separated
              << " edge_cross_axis_separated="
              << filters.edge_cross_axis_separated
              << " exact_remaining="
              << filters.exact_remaining
              << " filter_hash=" << filters.category_hash
              << " geometry_evaluation_us="
              << census.geometry_evaluation_us
              << " filter_census_us="
              << census.filter_census_us
              << " rerun_filter_census_us="
              << census.rerun_filter_census_us
              << " complete_device_keys=1"
              << " sorted_unique_keys=1"
              << " no_self_or_reversed_keys=1"
              << " deterministic_rerun=1"
              << " deterministic_filter_rerun=1"
              << " host_pair_bytes="
              << census.forecast.pair_key_host_bytes
              << " census_workspace_host="
              << census.forecast.fixed_workspace_host_bytes
              << " broadphase_owned_host="
              << census.forecast.exact.owned_host_bytes
              << " broadphase_startup_host="
              << census.forecast.exact.startup_host_bytes
              << " broadphase_sort_temp="
              << census.forecast.exact.sort_temp_bytes
              << " broadphase_scan_temp="
              << census.forecast.exact.scan_temp_bytes
              << " broadphase_pair_sort_temp="
              << census.forecast.exact.pair_sort_temp_bytes
              << " census_peak_host="
              << census.forecast.peak_census_host_reservation_bytes
              << " retained_setup_host_reservation="
              << census.forecast.retained_setup_host_reservation_bytes
              << " dynamics_peak_host_upper_bound="
              << census.forecast.physical_dynamics_peak_host_upper_bound
              << " broadphase_device="
              << census.forecast.exact.device_bytes
              << " owner_device="
              << census.forecast.physical_owner_device_bytes
              << " total_explicit_device_peak="
              << census.forecast.peak_total_explicit_device_bytes
              << " feature_discovery=0"
              << " intersection_processing=0"
              << " force_admission=0"
              << " interval_crossing=0"
              << '\n';
    RecordProperty("required_parent_pairs",
        std::to_string(census.probe_required_pairs));
    RecordProperty("q4_q4_parent_pairs",
        std::to_string(capacity.q4_q4_parent_pairs));
    RecordProperty("q4_t3_parent_pairs",
        std::to_string(capacity.q4_t3_parent_pairs));
    RecordProperty("t3_t3_parent_pairs",
        std::to_string(capacity.t3_t3_parent_pairs));
    RecordProperty("level0_facet_pairs",
        std::to_string(capacity.level0_facet_pairs));
    RecordProperty("pair_key_hash",
        std::to_string(capacity.pair_key_hash));
    RecordProperty("source_hash",
        std::to_string(capacity.surface_active_source_hash));
    RecordProperty("excluded_same_rigid_group",
        std::to_string(filters.excluded_same_rigid_group));
    RecordProperty("coordinate_aabb_separated",
        std::to_string(filters.coordinate_aabb_separated));
    RecordProperty("face_axis_separated",
        std::to_string(filters.face_axis_separated));
    RecordProperty("edge_cross_axis_separated",
        std::to_string(filters.edge_cross_axis_separated));
    RecordProperty("exact_remaining",
        std::to_string(filters.exact_remaining));
    RecordProperty("filter_hash",
        std::to_string(filters.category_hash));
    RecordProperty("geometry_evaluation_us",
        std::to_string(census.geometry_evaluation_us));
    RecordProperty("filter_census_us",
        std::to_string(census.filter_census_us));
    RecordProperty("broadphase_device_bytes",
        std::to_string(census.forecast.exact.device_bytes));
    RecordProperty("census_peak_host_reservation_bytes",
        std::to_string(
            census.forecast.peak_census_host_reservation_bytes));
    RecordProperty("peak_total_explicit_device_bytes",
        std::to_string(
            census.forecast.peak_total_explicit_device_bytes));
}

}  // namespace
}  // namespace crash::cases::vehicle_startup::shell_execution::self_contact_test
