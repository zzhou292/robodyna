#include "Fixture.h"
#include "../vehicle_shell_host/VehicleShellFixture.h"

namespace nodal_domain_test {
TEST(NodalDomainSize, DeclaredMaximum) {
  const auto limits=fe::NodalDomainLimits::Vehicle();
  std::vector<fe::NodalDomainNode> nodes(limits.max_nodes);
  for(std::size_t n=0;n<nodes.size();++n) nodes[n]={(std::uint64_t{1}<<61)+nodes.size()-n,{double(n),0,-0.}};
  fe::NodalNodeDomain domain; const auto empty=Bytes(domain);
  const auto last=nodes.back().source_id;
  nodes.back().source_id=nodes.front().source_id;
  auto report=domain.Initialize({91,nodes.data(),nodes.size()},limits);
  EXPECT_EQ(report.status,S::DuplicateIdentity); EXPECT_EQ(report.node,nodes.size()-1); EXPECT_EQ(Bytes(domain),empty);
  nodes.back().source_id=last;
  ASSERT_EQ(domain.Initialize({91,nodes.data(),nodes.size()},limits).status,S::Success);
  for(std::size_t n=0;n<nodes.size();++n) ASSERT_EQ(domain.Find(nodes[n].source_id),n);
  EXPECT_LE(domain.owned_payload_bytes(),limits.max_host_bytes);
  RecordProperty("nodes",nodes.size()); RecordProperty("owned_bytes",domain.owned_payload_bytes());
}

TEST(NodalDomainSize, SourceCountShellMap) {
  // Synthetic native geometry at actual no-tire shell counts. This proves map
  // capacity and exact retained identity, not original-source/mechanical closure.
  using F=vehicle_shell_test::Fixture;
  F source(F::SourceQ,F::SourceT,F::SourceNodes);
  fe::ShellBatchBinding binding;
  ASSERT_EQ(binding.Initialize(source.input(),fe::ShellHostBindingLimits::Vehicle()).status,fe::ShellBindingStatus::Success);
  auto nodes=Nodes(binding);
  nodes.insert(nodes.begin(),{UINT64_MAX,{1,2,3}});
  nodes.push_back({UINT64_MAX-1,{4,5,6}});
  auto domain=Domain(nodes); fe::ShellNodeMap map;
  ASSERT_EQ(map.Initialize(binding,domain,fe::ShellNodeMapLimits::Vehicle()).status,S::Success);
  EXPECT_FALSE(map.identity_map()); EXPECT_EQ(map.owner_node_count(),F::SourceNodes+2);
  for(std::size_t n=0;n<F::SourceNodes;++n) ASSERT_EQ(map.owner_index(n),n+1);
  EXPECT_EQ(map.shells()->inventory(),binding.inventory());
  EXPECT_TRUE(map.domain()->Matches(domain));
  RecordProperty("shell_nodes",F::SourceNodes);
  RecordProperty("owner_nodes",map.owner_node_count());
  RecordProperty("complete_map_payload_bytes",map.owned_payload_bytes());
}
} // namespace nodal_domain_test
