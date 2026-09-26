#include "../Internal.h"
#include "case/vehicle_startup/physical_model/tests/supports/Support.h"
#include <cstdlib>
namespace crash::cases::vehicle_self_contact::native::coated::test {
namespace physical = vehicle_startup::physical_model::supports_test;
namespace {
const selection::OriginalSelection& Selection() {
    static const auto value = [] {
        const auto* auxiliary = std::getenv("ROBO_SELF_CONTACT_AUX_MEMBER");
        const auto* combine = std::getenv("ROBO_SELF_CONTACT_COMBINE_MEMBER");
        output::Require(auxiliary && combine, "Missing authenticated original contact selection members");
        return selection::OriginalSelection::Prepare(modelio::vehicle::test::Canonical(),
            output::ReadBounded(auxiliary, selection::Limits{}.auxiliary_member_bytes),
            output::ReadBounded(combine, selection::Limits{}.combine_member_bytes));
    }();
    return value;
}
void SourceCounts() {
    const auto& model = physical::Model();
    ASSERT_EQ(model.source_domain().domain().node_count(), 376930u);
    ASSERT_EQ(model.shell_source().references().rows().size(), 349645u);
    ASSERT_EQ(model.source_domain().source().solid_source().data().rows.size(), 4980u);
    ASSERT_EQ(Selection().data().counts.retained_shells, 337092u);
    ASSERT_EQ(Selection().canonical().data().canonical_nodes, 393165u);
}
void Write(const char* name, const output::Document& document) {
    const auto* path = std::getenv("ROBO_V5_COATED_OUTPUT");
    output::Require(path && *path, "Missing create-only V5 coated assessment output");
    const std::filesystem::path directory(path);
    output::Require(std::filesystem::create_directory(directory), "V5 coating output directory already exists");
    output::WriteJson(directory/name, document);
    const auto bytes = output::ReadBounded(directory/name, 1u<<20);
    output::Document receipt; receipt.SetObject();
    output::String(receipt, "schema", "robo_dyna.v5_coated_assessment_receipt.v1");
    output::String(receipt, "scope", "retained V5 source-only assessment; no runtime or original full-case equivalence");
    output::String(receipt, "file", name); output::String(receipt, "sha256", output::Sha256(bytes));
    output::Integer(receipt, "bytes", bytes.size());
    const auto& in = Selection().canonical().data().inputs;
    output::String(receipt, "canonical_manifest_sha256", in.canonical_manifest.sha256);
    output::String(receipt, "scope_report_sha256", in.scope_report.sha256);
    output::String(receipt, "source_member_sha256", in.source_member.sha256);
    output::WriteJson(directory/"manifest.json", receipt);
}
}
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
