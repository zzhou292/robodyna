#include "Fixture.h"
#include "OriginalFixture.h"
#include "../qbat/source_fixture/YarisQbatSourceFixture.h"

namespace qbat_binding_test {
TEST(QbatBindingOriginal, Complete4250QuadsAndOneOriginalT3RetainIdentityAndEveryNodeContribution) {
  const OriginalCollection input;
  Binding binding;
  EXPECT_EQ(binding.InitializeFormulations(input.Input()).status,Status::InvalidInput);
  ASSERT_EQ(binding.InitializeFormulations(input.Input(),fe::ShellHostBindingLimits::Vehicle()).status,Status::Success);
  EXPECT_EQ(binding.qeph_count(),0u);
  EXPECT_EQ(binding.t3_count(),1u);
  EXPECT_EQ(binding.qbat_count(),4250u);
  EXPECT_EQ(binding.t3_source_id(0),2357656u);
  ASSERT_EQ(binding.node_count(),4384u);
  namespace original=yaris_qbat_source_fixture;
  for(std::size_t n=0;n<binding.node_count();++n) {
    const auto& actual=binding.nodes()[n];
    const auto& expected=original::nodes[n];
    EXPECT_EQ(actual.source_id,expected.id)<<n;
    EXPECT_EQ(Bits(actual.position.x),Bits(expected.position_m[0]))<<n;
    EXPECT_EQ(Bits(actual.position.y),Bits(expected.position_m[1]))<<n;
    EXPECT_EQ(Bits(actual.position.z),Bits(expected.position_m[2]))<<n;
  }
  for(std::size_t i=0;i<input.quads.size();++i) {
    EXPECT_EQ(binding.qbat_source_id(i),original::quads[i].id)<<i;
    EXPECT_EQ(binding.qbat_nodes(i),input.quads[i].nodes)<<i;
    EXPECT_TRUE(binding.qbat_reference(i).prepared())<<i;
  }
  Reduction(binding);
  EXPECT_EQ(binding.inventory().words().size(),5+23+47*4250u);
  RecordProperty("owned_host_bytes",std::to_string(binding.host_bytes()));
  RecordProperty("startup_scratch_bytes",std::to_string(binding.startup_scratch_bytes()));
}
TEST(QbatBindingOriginal, LastOriginalParentFailurePreservesDestinationThenExactBudgetRetry) {
  OriginalCollection input;
  Binding measured;
  ASSERT_EQ(measured.InitializeFormulations(input.Input(),fe::ShellHostBindingLimits::Vehicle()).status,Status::Success);
  Binding output;
  const auto before=Bytes(output);
  input.quads.back().reference.quadrilateral.position[3]=input.quads.back().reference.quadrilateral.position[1];
  const auto report=output.InitializeFormulations(input.Input(),fe::ShellHostBindingLimits::Vehicle());
  EXPECT_EQ(report.status,Status::InvalidQbatReference);
  EXPECT_EQ(report.family,fe::ShellBindingFamily::Qbat);
  EXPECT_EQ(report.parent_index,4249u);
  EXPECT_EQ(before,Bytes(output));
  const OriginalCollection clean;
  auto limits=fe::ShellHostBindingLimits::Vehicle();
  limits.max_owned_bytes=measured.host_bytes()-1;
  EXPECT_EQ(output.InitializeFormulations(clean.Input(),limits).status,Status::ResourceLimit);
  EXPECT_EQ(before,Bytes(output));
  ++limits.max_owned_bytes;
  limits.max_startup_scratch_bytes=measured.startup_scratch_bytes();
  ASSERT_EQ(output.InitializeFormulations(clean.Input(),limits).status,Status::Success);
  EXPECT_EQ(output.inventory(),measured.inventory());
  for(std::size_t n=0;n<output.node_count();++n) Exact(output.nodes()[n].native,measured.nodes()[n].native);
}
} // namespace qbat_binding_test
