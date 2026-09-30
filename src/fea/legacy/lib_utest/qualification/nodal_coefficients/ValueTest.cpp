#include "Fixture.h"
#include <type_traits>

namespace coefficient_test {
static_assert(std::is_nothrow_copy_constructible_v<fe::NodalCoefficientLedger>);
static_assert(!std::is_copy_assignable_v<fe::NodalCoefficientLedger>);
TEST(NodalCoefficients, CompleteTypedPartitionsCoverageAndNumericalFloorOnce) {
  const Fixture f;
  const auto d=f.Domain(); const auto map=f.Map(d);
  const auto s=f.Springs(); const auto b=f.Contributions(d);
  fe::NodalCoefficientLedger ledger;
  ASSERT_TRUE(ledger.Initialize({&map,&s,&b}));
  EXPECT_EQ(ledger.order(),fe::CoefficientOrder::PreparedSI_Q_T_B_Type25_Type13_V1);
  EXPECT_EQ(ledger.scope().qeph_parents,2); EXPECT_EQ(ledger.scope().t3_parents,1);
  EXPECT_EQ(ledger.scope().qbat_parents,1); EXPECT_EQ(ledger.scope().covered_nodes,6);
  EXPECT_EQ(ledger.scope().uncovered_nodes,1);
  EXPECT_EQ(ledger.scope().occurrences.qeph,8); EXPECT_EQ(ledger.scope().occurrences.t3,3);
  EXPECT_EQ(ledger.scope().occurrences.qbat,4); EXPECT_EQ(ledger.scope().occurrences.type25,4);
  EXPECT_EQ(ledger.scope().occurrences.type13,4);
  for(double value:Values(ledger.nodes()[2].coefficients)) EXPECT_EQ(Bits(value),Bits(0.));
  for(std::size_t local=0;local<f.shells.node_count();++local)
    qbat_binding_test::Exact(ledger.nodes()[f.map[local]].coefficients.shell,f.shells.nodes()[local].native);
  for(const auto& row:ledger.nodes()) {
    const auto& c=row.coefficients;
    EXPECT_EQ(Bits(c.type13.isotropic_inertia),Bits(c.type13.added_inertia));
  }
  EXPECT_EQ(ledger.type13()->model()->nodes()[3].source_id,99);
  EXPECT_EQ(ledger.domain()->Find(99),SIZE_MAX);
  EXPECT_GT(ledger.nodes()[6].coefficients.mass,0);
  EXPECT_EQ(ledger.nodes()[6].coefficients.shell.mass,0);
  EXPECT_EQ(ledger.nodes()[6].occurrences.type13,1);
}

TEST(NodalCoefficients, LegacyIdentityMapNodalCoefficientsRemainBitwiseEqual) {
  Fixture f;
  f.nodes.clear();
  for(const auto& node:f.shells.active_nodes()) f.nodes.push_back({node.source_id,node.position});
  for(auto& c:f.spring_input) for(unsigned local=0;local<2;++local) {
    const auto n=local?4:1;
    c.global_node[local]=n; c.source_node_id[local]=f.nodes[n].source_id;
    c.position[local]=f.nodes[n].position;
  }
  const auto d=f.Domain(); const auto map=f.Map(d); const auto s=f.Springs();
  ASSERT_TRUE(map.identity_map());
  fe::NodalMassBinding legacy; ASSERT_TRUE(legacy.Initialize(f.shells,s));
  fe::NodalCoefficientLedger ledger; ASSERT_TRUE(ledger.Initialize({&map,&s}));
  for(std::size_t n=0;n<d.node_count();++n) {
    const auto& old=legacy.nodes()[n].coefficients;
    const auto& c=ledger.nodes()[n].coefficients;
    EXPECT_EQ(Bits(c.mass),Bits(old.mass));
    EXPECT_EQ(Bits(c.isotropic_inertia),Bits(old.isotropic_inertia));
    EXPECT_EQ(Bits(c.type25.mass),Bits(old.connector_mass));
    EXPECT_EQ(Bits(c.type25.isotropic_inertia),Bits(old.connector_inertia));
    qbat_binding_test::Exact(c.shell,old.shell);
  }
  EXPECT_EQ(Bits(ledger.totals().mass),Bits(legacy.totals().mass));
  EXPECT_EQ(Bits(ledger.totals().isotropic_inertia),Bits(legacy.totals().isotropic_inertia));
}

TEST(NodalCoefficients, LateSourceFailuresPreserveEmptyLedgerAndExactRetry) {
  Fixture f; const auto d=f.Domain(); const auto map=f.Map(d);
  const auto s=f.Springs(); const auto b=f.Contributions(d);
  fe::NodalCoefficientLedger ledger; const auto before=Bytes(ledger);
  f.beam_input.connections.back().source_id=103;
  const auto duplicate=f.Contributions(d);
  auto report=ledger.Initialize({&map,&s,&duplicate});
  EXPECT_EQ(report.status,S::DuplicateIdentity); EXPECT_EQ(report.producer,fe::CoefficientProducer::Type13);
  EXPECT_EQ(report.parent,1); EXPECT_EQ(Bytes(ledger),before);
  f.spring_input.back().source_node_id[1]+=500;
  const auto wrong_id=f.Springs();
  report=ledger.Initialize({&map,&wrong_id,&b});
  EXPECT_EQ(report.status,S::IdentityMismatch); EXPECT_EQ(report.parent,1);
  EXPECT_EQ(report.local,1); EXPECT_EQ(Bytes(ledger),before);
  f.spring_input.back().source_node_id[1]-=500;
  f.spring_input.back().position[1].z=std::nextafter(0.,1.);
  const auto wrong_position=f.Springs();
  EXPECT_EQ(ledger.Initialize({&map,&wrong_position,&b}).status,S::PositionMismatch);
  const auto wrong_source=f.Springs(2);
  EXPECT_EQ(ledger.Initialize({&map,&wrong_source,&b}).status,S::IdentityMismatch);
  ASSERT_TRUE(ledger.Initialize({&map,&s,&b}));
  fe::NodalCoefficientLedger clean; ASSERT_TRUE(clean.Initialize({&map,&s,&b})); Exact(ledger,clean);
  const auto published=Bytes(ledger);
  EXPECT_EQ(ledger.Initialize({}).status,S::AlreadyInitialized); EXPECT_EQ(Bytes(ledger),published);
}

TEST(NodalCoefficients, CompletePeakCapsSharedBackingAndOwnedLifetime) {
  const Fixture f; const auto d=f.Domain(); const auto map=f.Map(d);
  const auto s=f.Springs(); const auto b=f.Contributions(d);
  fe::NodalCoefficientLedger ledger; ASSERT_TRUE(ledger.Initialize({&map,&s,&b}));
  auto limits=fe::CoefficientLimits::Vehicle(); limits.max_host_bytes=ledger.startup_payload_bytes()-1;
  fe::NodalCoefficientLedger retry;
  EXPECT_EQ(retry.Initialize({&map,&s,&b},limits).status,S::ResourceLimit);
  ++limits.max_host_bytes; ASSERT_TRUE(retry.Initialize({&map,&s,&b},limits)); Exact(ledger,retry);
  const auto independent=f.Domain(); ASSERT_TRUE(d.Matches(independent)); ASSERT_FALSE(d.SharesStorage(independent));
  const auto independent_b=f.Contributions(independent);
  fe::NodalCoefficientLedger separate;
  EXPECT_EQ(separate.Initialize({&map,&s,&independent_b},limits).status,S::ResourceLimit);
  ASSERT_TRUE(separate.Initialize({&map,&s,&independent_b}));
  EXPECT_EQ(separate.owned_payload_bytes()-ledger.owned_payload_bytes(),d.owned_payload_bytes()-sizeof(d));
  EXPECT_TRUE(separate.Matches(ledger)); Exact(separate,ledger);
  const auto surviving=[] {
    const Fixture x; const auto domain=x.Domain(); const auto m=x.Map(domain);
    const auto springs=x.Springs(); const auto beams=x.Contributions(domain);
    fe::NodalCoefficientLedger out; EXPECT_TRUE(out.Initialize({&m,&springs,&beams})); return out;
  }();
  EXPECT_TRUE(surviving.Matches(ledger)); Exact(surviving,ledger);
  fe::NodalCoefficientLedger moved(std::move(surviving)); EXPECT_TRUE(surviving.prepared());
  EXPECT_TRUE(moved.Matches(ledger)); EXPECT_FALSE(moved.Matches({&map,&s}));
}

TEST(NodalCoefficients, LimitsEmptySourcesAndSignedZeroAssociation) {
  Fixture f; const auto d=f.Domain(); const auto map=f.Map(d); const auto s=f.Springs();
  fe::NodalCoefficientLedger ledger; const auto before=Bytes(ledger);
  auto limits=fe::CoefficientLimits::Vehicle(); limits.max_shell_parents=3;
  EXPECT_EQ(ledger.Initialize({&map,&s},limits).status,S::ResourceLimit);
  limits=fe::CoefficientLimits::Vehicle(); limits.max_nodes=6;
  EXPECT_EQ(ledger.Initialize({&map,&s},limits).status,S::ResourceLimit);
  limits=fe::CoefficientLimits::Vehicle(); limits.max_type25_connections=1;
  EXPECT_EQ(ledger.Initialize({&map,&s},limits).status,S::ResourceLimit);
  EXPECT_EQ(ledger.Initialize({}).status,S::InvalidInput);
  spring::Model empty; EXPECT_EQ(ledger.Initialize({&map,&empty}).status,S::InvalidInput);
  f.spring_input[1].position[1].y=-0.; const auto wrong=f.Springs();
  EXPECT_EQ(ledger.Initialize({&map,&wrong}).status,S::PositionMismatch);
  EXPECT_EQ(Bytes(ledger),before);
  ASSERT_TRUE(ledger.Initialize({&map})); EXPECT_EQ(ledger.scope().uncovered_nodes,2);
  EXPECT_EQ(ledger.type25(),nullptr); EXPECT_EQ(ledger.type13(),nullptr);
}
} // namespace coefficient_test
