#include "Fixture.h"
#include <gtest/gtest.h>
namespace crash::cases::vehicle_self_contact::native::test {
TEST(NativeTopologyAssessment, CompleteCanonicalDomainAndOriginalDispositionsAreRetained) {
  Fixture f;const auto before=f.Get<std::uint64_t>("node_ids");const auto input=f.Inputs();
  EXPECT_EQ(input.node_ids,before);ASSERT_EQ(input.node_ids.size(),12u);
  EXPECT_EQ(input.node_ids.back(),120u);EXPECT_EQ(input.positions.size(),36u);ASSERT_EQ(input.primary.size(),2u);
  EXPECT_EQ(input.primary[0].source_id,5010u);EXPECT_EQ(input.primary[1].source_id,5011u);
  EXPECT_EQ(input.primary[1].nodes[2],input.primary[1].nodes[3]);
  EXPECT_EQ(input.View().coordinates,s::Coordinates::Si);EXPECT_EQ(input.View().units.length_m,.001);
  const auto result=d::EvaluateValues(f.canonical,f.selected,{},{});ASSERT_TRUE(result.output_complete);
  EXPECT_EQ(result.counts.canonical_nodes,12u);EXPECT_EQ(result.counts.selected_parents,2u);
  EXPECT_EQ(result.counts.selected_q4,1u);EXPECT_EQ(result.counts.selected_t3,1u);
  EXPECT_EQ(result.parts.size(),4u);EXPECT_EQ(result.counts.original.solids,1u);EXPECT_EQ(result.counts.original.beams,1u);
  EXPECT_EQ(result.counts.output_mains,4u);EXPECT_EQ(result.output_digest.fields.size(),15u);
  const auto doc=ResultDocument(result);EXPECT_TRUE(doc["output_complete"].GetBool());
  EXPECT_FALSE(doc["physical_shell_formulations_admitted"].GetBool());
  EXPECT_EQ(f.Get<std::uint64_t>("node_ids"),before);
}
TEST(NativeTopologyAssessment, CanonicalRecordOrderAndExplicitRetainedPolicyAreNotPhysicalFiltering) {
  Fixture f;const auto first=f.Inputs();auto rows=f.Get<std::uint64_t>("shells_records");
  auto nodes=f.Get<std::uint32_t>("shells_node_indices");auto lines=f.Get<std::uint32_t>("shells_source_lines");
  for(unsigned k=0;k<6;++k)std::swap(rows[k],rows[6+k]);
  for(unsigned k=0;k<4;++k)std::swap(nodes[k],nodes[4+k]);std::swap(lines[0],lines[1]);
  f.Set("shells_records",rows,6);f.Set("shells_node_indices",nodes,4);f.Set("shells_source_lines",lines,1);
  const auto reversed=f.Inputs();EXPECT_EQ(reversed.node_ids,first.node_ids);
  EXPECT_EQ(reversed.primary[0].source_id,5011u);EXPECT_EQ(reversed.origin[0].canonical_row,0u);EXPECT_EQ(reversed.origin[0].source_line,102u);
  EXPECT_NE(d::InputDigest(first,{},1u<<20).sha256,d::InputDigest(reversed,{},1u<<20).sha256);
  f.canonical.selected_parts={100};f.canonical.excluded_parts={101};f.selected.parts[1].retained_shell_part=false;f.selected.parts[1].excluded_shell_part=true;
  f.selected.counts.retained_shell_parts=1;f.selected.counts.excluded_shell_parts=1;f.selected.counts.retained_shells=1;f.selected.counts.excluded_shells=1;
  const auto selected=f.Inputs();ASSERT_EQ(selected.primary.size(),1u);EXPECT_EQ(selected.primary[0].source_id,5010u);EXPECT_EQ(selected.node_ids.size(),12u);
}
TEST(NativeTopologyAssessment, SourceMismatchCapsAndCountOnlyForecastRejectBeforeTopology) {
  Fixture f;auto cap=Limits{};const auto plan=d::Plan(f.canonical,f.selected,{},cap);ASSERT_TRUE(plan.admitted);
  auto small_metadata=cap;small_metadata.metadata_bytes=64;
  EXPECT_EQ(d::Plan(f.canonical,f.selected,{},small_metadata).result_owned_bytes,plan.result_owned_bytes);
  EXPECT_GE(plan.result_owned_bytes,sizeof(Result)+f.selected.parts.size()*sizeof(selection::PartDisposition));
  EXPECT_EQ(plan.main_size,sizeof(s::Main));EXPECT_EQ(plan.reference_size,sizeof(s::NormalReference));
  EXPECT_EQ(plan.topology.ready_output_bytes,0u);EXPECT_EQ(plan.topology.ready_scratch_bytes,0u);
  cap.host_bytes=plan.peak_host_bytes-1;EXPECT_FALSE(d::Plan(f.canonical,f.selected,{},cap).admitted);
  EXPECT_THROW(d::EvaluateValues(f.canonical,f.selected,{},cap),std::exception);
  ++cap.host_bytes;EXPECT_TRUE(d::Plan(f.canonical,f.selected,{},cap).admitted);
  Config invalid;invalid.order=static_cast<Order>(99);
  EXPECT_THROW(d::Plan(f.canonical,f.selected,invalid,{}),std::exception);
  auto bad=f.Get<std::uint32_t>("shells_node_indices");bad[0]=11;f.Set("shells_node_indices",bad,4);
  EXPECT_THROW(f.Inputs(),std::exception);
}
TEST(NativeTopologyAssessment, WholeTopologyRejectionReportsExactSourceWithoutDeletionRetry) {
  Fixture f;auto rows=f.Get<std::uint64_t>("shells_records");auto nodes=f.Get<std::uint32_t>("shells_node_indices");
  for(unsigned k=0;k<4;++k){rows[8+k]=rows[2+k];nodes[4+k]=nodes[k];}
  f.Set("shells_records",rows,6);f.Set("shells_node_indices",nodes,4);
  const auto result=d::EvaluateValues(f.canonical,f.selected,{},{});
  EXPECT_FALSE(result.output_complete);EXPECT_EQ(result.topology_report.status,s::Status::UnsupportedTopology);
  EXPECT_EQ(result.counts.selected_parents,2u);EXPECT_EQ(result.counts.canonical_nodes,12u);EXPECT_EQ(result.output_digest.sha256,"");
  EXPECT_EQ(result.failure.source_eid,5011u);EXPECT_EQ(result.failure.source_pid,101u);EXPECT_EQ(result.failure.canonical_row,1u);EXPECT_EQ(result.failure.source_line,102u);
  const auto doc=ResultDocument(result);EXPECT_TRUE(doc["output_digest"].IsNull());EXPECT_FALSE(doc["geometry_deletion_retry"].GetBool());
  EXPECT_EQ(f.Get<std::uint64_t>("shells_records"),rows);
}
TEST(NativeTopologyAssessment, DocumentsRespectMetadataCapacity) {
  Fixture f;const auto result=d::EvaluateValues(f.canonical,f.selected,{},{});
  EXPECT_THROW(ResultDocument(result,64),std::exception);
  EXPECT_THROW(ForecastDocument(result.forecast,64),std::exception);
  EXPECT_NO_THROW(ResultDocument(result));
  EXPECT_NO_THROW(ForecastDocument(result.forecast));
}
} // namespace crash::cases::vehicle_self_contact::native::test
