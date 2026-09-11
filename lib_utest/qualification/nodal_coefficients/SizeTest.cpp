#include "Fixture.h"
#include "../vehicle_shell_host/VehicleShellFixture.h"

namespace coefficient_test {
TEST(NodalCoefficientSize, ActualSourceCountsSyntheticShellsAndUncoveredExtras) {
  using F=vehicle_shell_test::Fixture;
  F source(F::SourceQ,F::SourceT,F::SourceNodes);
  constexpr std::size_t qbat_count=4250;
  std::vector<fe::ShellQbatBindingInput> qbat(qbat_count);
  for(std::size_t e=0;e<qbat_count;++e) {
    const auto& q=source.q[source.q.size()-qbat_count+e];
    qbat[e].nodes=q.nodes;
    qbat[e].source_parent_id=q.source_parent_id;
    qbat[e].reference.quadrilateral=q.reference;
    qbat[e].reference.initial_a11_pa=q.reference.young_modulus/
        (1-q.reference.poisson_ratio*q.reference.poisson_ratio);
  }
  source.q.resize(source.q.size()-qbat_count);
  fe::ShellBatchBinding binding;
  ASSERT_EQ(binding.InitializeFormulations({source.input(),qbat.data(),qbat.size()},
      fe::ShellHostBindingLimits::Vehicle()).status,fe::ShellBindingStatus::Success);
  std::vector<fe::NodalDomainNode> nodes{{UINT64_MAX,{1,2,3}}};
  for(const auto& n:binding.active_nodes()) nodes.push_back({n.source_id,n.position});
  nodes.push_back({UINT64_MAX-1,{4,5,6}});
  fe::NodalNodeDomain domain;
  ASSERT_TRUE(domain.Initialize({17,nodes.data(),nodes.size()},fe::NodalDomainLimits::Vehicle()));
  fe::ShellNodeMap map;
  ASSERT_TRUE(map.Initialize(binding,domain,fe::ShellNodeMapLimits::Vehicle()));
  fe::NodalCoefficientLedger ledger;
  ASSERT_TRUE(ledger.Initialize({&map},fe::CoefficientLimits::Vehicle()));
  EXPECT_EQ(ledger.scope().covered_nodes,F::SourceNodes);
  EXPECT_EQ(ledger.scope().uncovered_nodes,2);
  EXPECT_EQ(ledger.scope().qeph_parents,F::SourceQ-qbat_count);
  EXPECT_EQ(ledger.scope().t3_parents,F::SourceT);
  EXPECT_EQ(ledger.scope().qbat_parents,qbat_count);
  for(std::size_t n=0;n<F::SourceNodes;++n) {
    const auto& actual=ledger.nodes()[n+1].coefficients;
    ASSERT_EQ(Bits(actual.mass),Bits(binding.nodes()[n].native.mass));
    ASSERT_EQ(Bits(actual.isotropic_inertia),Bits(binding.nodes()[n].native.isotropic_inertia));
    qbat_binding_test::Exact(actual.shell,binding.nodes()[n].native);
  }
  EXPECT_EQ(ledger.nodes()[0].coefficients.mass,0);
  EXPECT_EQ(ledger.nodes()[nodes.size()-1].coefficients.mass,0);
  EXPECT_LE(ledger.startup_payload_bytes(),fe::CoefficientLimits::Vehicle().max_host_bytes);
  RecordProperty("node_count",nodes.size());
  RecordProperty("retained_bytes",ledger.owned_payload_bytes());
  RecordProperty("startup_bytes",ledger.startup_payload_bytes());
}
} // namespace coefficient_test
