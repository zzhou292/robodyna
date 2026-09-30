// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
namespace beam18_model_test {
TEST(Beam18Ledger, FixedAppendOrderCompleteOccurrencesAndUncoveredZero) {
  const Fixture f; const auto d=f.structural.Domain(); const auto map=f.structural.Map(d);
  const auto type25=f.structural.Springs(); const auto type13=f.structural.Contributions(d); const auto beam=f.Contributions(d);
  fe::NodalCoefficientSources sources{&map,&type25,&type13};
  fe::NodalCoefficientLedger old,joined; ASSERT_TRUE(old.Initialize(sources));
  ASSERT_TRUE(joined.InitializeWithBeams({{sources},&beam}));
  EXPECT_EQ(joined.order(),fe::CoefficientOrder::PreparedSI_Q_T_B_Type25_Type13_ElementMass_Solid18_24_6z_Law44_Law90_Beam18_V5);
  EXPECT_EQ(joined.scope().beam18_parents,2); EXPECT_EQ(joined.scope().occurrences.beam18,4);
  EXPECT_EQ(joined.scope().uncovered_nodes,1); EXPECT_FALSE(joined.Matches(sources));
  EXPECT_TRUE(joined.MatchesWithBeams({{sources},&beam}));
  for(std::size_t n=0;n<d.node_count();++n) {
    double mass=old.nodes()[n].coefficients.mass,inertia=old.nodes()[n].coefficients.isotropic_inertia;
    double bm=0,bj=0; unsigned occurrences=0;
    for(const auto& record:beam.records()) if(record.value.global_node==n) {
      const auto& c=record.value.coefficients;
      mass+=c.mass_kg; inertia+=c.native_total_inertia_kg_m2;
      bm+=c.mass_kg; bj+=c.native_total_inertia_kg_m2; ++occurrences;
    }
    const auto& row=joined.nodes()[n];
    EXPECT_EQ(Bits(row.coefficients.mass),Bits(mass)); EXPECT_EQ(Bits(row.coefficients.isotropic_inertia),Bits(inertia));
    EXPECT_EQ(Bits(row.coefficients.beam18.mass),Bits(bm)); EXPECT_EQ(Bits(row.coefficients.beam18.isotropic_inertia),Bits(bj));
    EXPECT_EQ(row.occurrences.beam18,occurrences);
  }
  EXPECT_FALSE(fe::HasCoefficientProducer(joined.nodes()[2].occurrences));
}
TEST(Beam18Ledger, LateStructuralDuplicateRejectsButIndependentMassAndWidNamespacesRemain) {
  Fixture f; const auto d=f.structural.Domain(); const auto map=f.structural.Map(d);
  const auto type25=f.structural.Springs(); auto raw=f.rows.back().reference.input();
  raw.source_element_id=100; // Existing shell EID, despite TYPE25 WID sharing same value legally.
  ASSERT_EQ(b::InitializeReference(raw,f.rows.back().reference),b::Status::Success);
  const auto duplicate=f.Contributions(d); fe::NodalCoefficientLedger out; const auto before=Bytes(out);
  auto r=out.InitializeWithBeams({{{&map,&type25}},&duplicate});
  EXPECT_EQ(r.status,fe::CoefficientStatus::DuplicateIdentity); EXPECT_EQ(r.producer,fe::CoefficientProducer::Beam18);
  EXPECT_EQ(r.parent,1); EXPECT_EQ(Bytes(out),before);
  raw.source_element_id=9993; ASSERT_EQ(b::InitializeReference(raw,f.rows.back().reference),b::Status::Success);
  const auto valid=f.Contributions(d); ASSERT_TRUE(out.InitializeWithBeams({{{&map,&type25}},&valid}));
}
TEST(Beam18Ledger, SharedDomainPeakOneByteShortIndependentBackingAndRetry) {
  const Fixture f; const auto d=f.structural.Domain(); const auto map=f.structural.Map(d); const auto beam=f.Contributions(d);
  fe::NodalCoefficientLedger first; ASSERT_TRUE(first.InitializeWithBeams({{{&map}},&beam}));
  auto cap=fe::CoefficientLimits::Vehicle(); cap.max_host_bytes=first.startup_payload_bytes()-1;
  fe::NodalCoefficientLedger retry; EXPECT_EQ(retry.InitializeWithBeams({{{&map}},&beam},cap).status,fe::CoefficientStatus::ResourceLimit);
  ++cap.max_host_bytes; ASSERT_TRUE(retry.InitializeWithBeams({{{&map}},&beam},cap)); EXPECT_TRUE(retry.Matches(first));
  const auto other_domain=f.structural.Domain(); const auto separate=f.Contributions(other_domain);
  fe::NodalCoefficientLedger extra; EXPECT_EQ(extra.InitializeWithBeams({{{&map}},&separate},cap).status,fe::CoefficientStatus::ResourceLimit);
  ASSERT_TRUE(extra.InitializeWithBeams({{{&map}},&separate})); EXPECT_TRUE(extra.Matches(first));
  EXPECT_EQ(extra.owned_payload_bytes()-first.owned_payload_bytes(),d.owned_payload_bytes()-sizeof(d));
  fe::NodalCoefficientLedger empty; EXPECT_EQ(empty.InitializeWithBeams({{{&map}},nullptr}).status,fe::CoefficientStatus::IdentityMismatch);
}
} // namespace beam18_model_test
