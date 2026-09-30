#include "../TopologyAssessment.h"
#include "modelio/vehicle_source/tests/TestSupport.h"
#include "output/BoundedArrayJson.h"
#include <cstdlib>
#include <iostream>
namespace crash::cases::vehicle_self_contact::native::test {
namespace {
const selection::OriginalSelection& Actual() {
  static const auto value=[] {
    const auto* auxiliary=std::getenv("ROBO_SELF_CONTACT_AUX_MEMBER");const auto* combine=std::getenv("ROBO_SELF_CONTACT_COMBINE_MEMBER");
    output::Require(auxiliary&&combine,"Missing explicit authenticated original contact members");
    return selection::OriginalSelection::Prepare(modelio::vehicle::test::Canonical(),
        output::ReadBounded(auxiliary,selection::Limits{}.auxiliary_member_bytes),
        output::ReadBounded(combine,selection::Limits{}.combine_member_bytes));
  }();return value;
}
void Write(const char* filename,const output::Document& doc) {
  const auto* name=std::getenv("ROBO_NATIVE_TOPOLOGY_OUTPUT");output::Require(name&&*name,"Missing create-only assessment output directory");
  const std::filesystem::path directory(name);output::Require(std::filesystem::create_directory(directory),"Assessment output already exists");
  output::WriteJson(directory/filename,doc);
  const auto bytes=output::ReadBounded(directory/filename,1u<<20);
  output::Document manifest;manifest.SetObject();output::String(manifest,"schema","robo_dyna.native_topology_assessment_receipt.v1");
  output::String(manifest,"scope","source assessment only;not physical/native full-case-order or GPU runtime admission");
  output::String(manifest,"file",filename);output::String(manifest,"sha256",output::Sha256(bytes));output::Integer(manifest,"bytes",bytes.size());
  const auto& inputs=Actual().canonical().data().inputs;
  output::String(manifest,"canonical_root",inputs.canonical_root.string());output::String(manifest,"scope_root",inputs.scope_root.string());
  output::String(manifest,"source_member_sha256",inputs.source_member.sha256);output::String(manifest,"canonical_manifest_sha256",inputs.canonical_manifest.sha256);
  output::String(manifest,"scope_report_sha256",inputs.scope_report.sha256);
  output::WriteJson(directory/"manifest.json",manifest);
}
void SourceCounts() {
  ASSERT_EQ(Actual().canonical().data().canonical_nodes,393165u);
  ASSERT_EQ(Actual().canonical().data().canonical_shells,358457u);
  ASSERT_EQ(Actual().data().counts.retained_shells,337092u);
  ASSERT_EQ(Actual().data().counts.retained_shell_parts,842u);
  ASSERT_EQ(Actual().data().counts.solids,2952u);
}
}
TEST(NativeTopologyAssessmentActual, ForecastCompleteSourceWithoutTopologyAllocation) {
  ASSERT_NO_FATAL_FAILURE(SourceCounts());
  const auto forecast=Preflight(Actual());const auto doc=ForecastDocument(forecast);Write("forecast.json",doc);
  EXPECT_TRUE(forecast.admitted);EXPECT_EQ(forecast.topology.expanded_mains,674184u);
  EXPECT_EQ(forecast.topology.maximum_references,2696736u);
  RecordProperty("peak_host_reservation_bytes",std::to_string(forecast.peak_host_bytes));
  RecordProperty("tl_output_bytes",std::to_string(forecast.topology.output_bytes));
  RecordProperty("tl_scratch_bytes",std::to_string(forecast.topology.scratch_bytes));
}
TEST(NativeTopologyAssessmentActual, CompleteSelectedCanonicalDomainHasOneReportedOutcome) {
  ASSERT_NO_FATAL_FAILURE(SourceCounts());
  const auto copied=Actual();EXPECT_EQ(&copied.canonical().data(),&Actual().canonical().data());
  const auto result=Assess(copied);Write("assessment.json",ResultDocument(result));
  EXPECT_EQ(result.counts.canonical_nodes,393165u);EXPECT_EQ(result.counts.selected_parents,337092u);
  EXPECT_EQ(result.counts.selected_q4,315963u);EXPECT_EQ(result.counts.selected_t3,21129u);
  // A source-bound rejected topology is a complete assessment outcome, not
  // geometry acceptance. Its document records false output_complete/no digest.
  EXPECT_TRUE(result.output_complete||result.failure.source_eid||result.failure.source_node_id);
  if(result.output_complete)EXPECT_EQ(result.counts.output_mains,674184u);
  RecordProperty("geometry_accepted",result.output_complete?1:0);
  RecordProperty("native_warning_count",std::to_string(result.topology_report.neighbor_warnings.count));
  std::cout<<"Assessment output_complete="<<result.output_complete<<" source_eid="<<result.failure.source_eid
           <<" source_node="<<result.failure.source_node_id<<" digest="<<result.output_digest.sha256<<'\n';
}
} // namespace crash::cases::vehicle_self_contact::native::test
