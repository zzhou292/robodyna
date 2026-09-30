#include "ActualFixture.h"
namespace crash::cases::vehicle_self_contact::native::coated::test {
TEST(V5CoatedSourceActual, ForecastCompleteRetainedModelBeforeNewSourceAllocation) {
    ASSERT_NO_FATAL_FAILURE(SourceCounts());
    const auto forecast = Preflight(physical::Model(), Selection());
    Write("forecast.json", ForecastDocument(forecast));
    EXPECT_TRUE(forecast.admitted);
    EXPECT_EQ(forecast.topology.expanded_mains, 674184u);
    RecordProperty("peak_reservation_bytes", std::to_string(forecast.peak_bytes));
    RecordProperty("retained_model_reservation", std::to_string(forecast.retained_model_reservation));
    auto limits = Limits{}; limits.host_bytes = forecast.peak_bytes - 1;
    EXPECT_FALSE(Preflight(physical::Model(), Selection(), {}, limits).admitted);
    // Admission precedes member decode or any new assessment allocation.
    EXPECT_THROW(AssessCoatedSource(physical::Model(), Selection(), "", {}, limits), std::exception);
}
TEST(V5CoatedSourceActual, CompletePhysicalMembershipAndSelectedTopologyHaveOneReportedOutcome) {
    ASSERT_NO_FATAL_FAILURE(SourceCounts());
    const auto result = AssessCoatedSource(physical::Model(), Selection(), physical::Inputs().member);
    Write("assessment.json", ResultDocument(result));
    EXPECT_EQ(result.physical.shells, 349645u); EXPECT_EQ(result.contact.shells, 337092u);
    EXPECT_EQ(result.physical_solids, 4980u); EXPECT_EQ(result.physical_nodes, 376930u);
    EXPECT_TRUE(result.physical_roles_complete || result.first_unready.source_eid);
    EXPECT_EQ(result.topology_attempted, result.selected_roles_complete);
    EXPECT_TRUE(!result.selected_roles_complete || result.topology_complete || result.failure.source_eid || result.failure.source_node_id);
    if (result.topology_complete) EXPECT_EQ(result.output_mains, 674184u);
    RecordProperty("physical_membership_pairs", std::to_string(result.physical.matches));
    RecordProperty("contact_membership_pairs", std::to_string(result.contact.matches));
    RecordProperty("contact_forward", std::to_string(result.contact.forward));
    RecordProperty("contact_reversed", std::to_string(result.contact.reversed));
    RecordProperty("input_digest", result.input_digest.sha256);
    RecordProperty("output_digest", result.output_digest.sha256);
    // Prior original-solid CSV filtered by retained PIDs: diagnostic comparator
    // only. No row, orientation or producer input is loaded from that census.
    RecordProperty("old_csv_pid_filter_pairs_crosscheck_only", 741);
}
} // namespace crash::cases::vehicle_self_contact::native::coated::test
