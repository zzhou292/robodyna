#include "Fixture.h"
#include "../type13_model/OriginalSource.h"
#include <algorithm>

namespace type13_contribution_test {
TEST(Type13ContributionsOriginal, All4442KeepEveryPreparedEndpointAndOriginalIdentity) {
  type13_model_test::Source source;
  // One unrelated interleaved owner node; source element/node order is intact.
  for(auto& node:source.nodes) if(node.global_node!=SIZE_MAX) ++node.global_node;
  auto input=source.Input(); ++input.global_node_count;
  t::Model model; ASSERT_TRUE(model.Initialize(input));
  auto nodes=Nodes(model); auto domain=Domain(nodes);
  fe::Type13NodeContributions adapter; ASSERT_TRUE(adapter.Initialize(model,domain));
  Check(adapter,model);
  ASSERT_EQ(adapter.record_count(),8884);
  std::vector<bool> observed(domain.node_count());
  for(std::size_t e=0;e<4442;++e) {
    const auto& original=type13_model_test::original::Beams[e];
    for(unsigned local=0;local<2;++local) {
      const auto& row=adapter.records()[2*e+local].value;
      EXPECT_EQ(row.source_element_id,original.id); EXPECT_EQ(row.source_property_id,2000486);
      EXPECT_EQ(row.source_node_id,type13_model_test::original::Nodes[original.nodes[local]].id);
      EXPECT_NE(row.source_node_id,2000001); ASSERT_GT(row.global_node,0);
      const auto& node=type13_model_test::original::Nodes[original.nodes[local]];
      const auto& position=domain.nodes()[row.global_node].position;
      EXPECT_EQ(Bytes(position.x),Bytes(node.si[0]));
      EXPECT_EQ(Bytes(position.y),Bytes(node.si[1]));
      EXPECT_EQ(Bytes(position.z),Bytes(node.si[2]));
      observed[row.global_node]=true;
    }
  }
  EXPECT_EQ(std::count(observed.begin(),observed.end(),true),7493);
  EXPECT_FALSE(observed[0]);
  EXPECT_EQ(adapter.model()->nodes()[0].global_node,SIZE_MAX);
  RecordProperty("endpoint_records",adapter.record_count());
  RecordProperty("owned_bytes",adapter.owned_payload_bytes());
}

TEST(Type13ContributionsOriginal, FinalOriginalSourceNodeAndExactBudgetRetry) {
  type13_model_test::Source source; t::Model model; ASSERT_TRUE(model.Initialize(source.Input()));
  auto nodes=Nodes(model); const auto domain=Domain(nodes);
  fe::Type13NodeContributions clean; ASSERT_TRUE(clean.Initialize(model,domain));
  const auto cap=clean.startup_payload_bytes();
  fe::Type13NodeContributions retry; const auto before=Bytes(retry);
  nodes.back().position.x=std::nextafter(nodes.back().position.x,1e6);
  auto changed=Domain(nodes);
  EXPECT_EQ(retry.Initialize(model,changed).status,S::PositionMismatch); EXPECT_EQ(Bytes(retry),before);
  fe::Type13ContributionLimits limits; limits.max_host_bytes=cap-1;
  EXPECT_EQ(retry.Initialize(model,domain,limits).status,S::ResourceLimit); EXPECT_EQ(Bytes(retry),before);
  ++limits.max_host_bytes; ASSERT_TRUE(retry.Initialize(model,domain,limits));
  EXPECT_TRUE(retry.Matches(clean)); Check(retry,model);
}
} // namespace type13_contribution_test
