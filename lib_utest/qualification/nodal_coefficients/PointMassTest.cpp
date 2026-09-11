#include "Fixture.h"

namespace coefficient_test {
namespace {
fe::ElementMassContributions Points(const fe::NodalNodeDomain& domain,
    const std::vector<fe::ElementMassSource>& rows,double scale=1000) {
  fe::ElementMassContributions result;
  const auto report=result.Initialize(domain,{domain.source_instance_id(),scale,rows.data(),rows.size()});
  EXPECT_TRUE(report)<<report.message;
  return result;
}
std::vector<fe::ElementMassSource> Rows(const Fixture& f) {
  return {{9100,f.nodes[2].source_id,2,.0011},
          {9101,f.nodes[f.map[1]].source_id,f.map[1],.0023},
          {9102,f.nodes[f.map[1]].source_id,f.map[1],0.0}};
}
}
TEST(ElementMassContributions, OwnedOriginalRowsDomainIdentityAndZeroScalarInertia) {
  Fixture f;
  const auto domain=f.Domain();
  auto rows=Rows(f);
  const auto points=Points(domain,rows);
  const auto copied=points;
  rows.back().mass_source=-0.0;
  const auto negative_zero=Points(domain,rows);
  EXPECT_FALSE(points.Matches(negative_zero));
  EXPECT_EQ(Bits(negative_zero.records()[2].mass_kg),Bits(-0.0));
  rows.front().source_element_id=9900;
  rows.front().mass_source=1;
  EXPECT_EQ(points.records()[0].source.source_element_id,9100u);
  EXPECT_EQ(Bits(points.records()[0].source.mass_source),Bits(.0011));
  EXPECT_EQ(Bits(points.records()[0].mass_kg),Bits(.0011*1000));
  EXPECT_EQ(Bits(points.records()[0].isotropic_inertia_kg_m2()),Bits(0.0));
  EXPECT_TRUE(copied.Matches(points));
  const auto separate_domain=f.Domain();
  EXPECT_TRUE(points.Matches(Points(separate_domain,Rows(f))));
  f.nodes[2].position.x=-0.0;
  EXPECT_FALSE(points.Matches(Points(f.Domain(),Rows(f))));
  const auto retained=[] {
    Fixture local;
    const auto d=local.Domain();
    return Points(d,Rows(local));
  }();
  EXPECT_TRUE(retained.Matches(points));
}
TEST(ElementMassContributions, LateRejectExactRetryAndBytesBeforeBorrowedRows) {
  const Fixture f;
  const auto domain=f.Domain();
  const auto clean=Rows(f);
  const auto expected=Points(domain,clean);
  using D=fe::NodalDomainStatus;
  for(unsigned kind=0;kind<6;++kind) {
    auto bad=clean;
    auto& tail=bad.back();
    if(kind==0) tail.source_element_id=bad.front().source_element_id;
    if(kind==1) tail.source_node_id=777;
    if(kind==2) tail.domain_node=domain.node_count();
    if(kind==3) tail.mass_source=-1;
    if(kind==4) tail.mass_source=std::numeric_limits<double>::quiet_NaN();
    if(kind==5) tail.mass_source=std::numeric_limits<double>::max();
    fe::ElementMassContributions result;
    const auto before=Bytes(result);
    const auto report=result.Initialize(domain,{1,1000,bad.data(),bad.size()});
    EXPECT_FALSE(report); EXPECT_EQ(report.node,2u); EXPECT_EQ(Bytes(result),before);
    ASSERT_TRUE(result.Initialize(domain,{1,1000,clean.data(),clean.size()}));
    EXPECT_TRUE(result.Matches(expected));
    EXPECT_EQ(result.Initialize(domain,{}).status,D::AlreadyInitialized);
  }
  const auto* poisoned=reinterpret_cast<const fe::ElementMassSource*>(alignof(fe::ElementMassSource));
  fe::ElementMassContributions result;
  EXPECT_EQ(result.Initialize(domain,{1,1000,poisoned,4097}).status,D::ResourceLimit);
  auto cap=fe::ElementMassLimits{};
  cap.max_host_bytes=expected.startup_payload_bytes()-1;
  EXPECT_EQ(result.Initialize(domain,{1,1000,poisoned,clean.size()},cap).status,D::ResourceLimit);
  ++cap.max_host_bytes;
  ASSERT_TRUE(result.Initialize(domain,{1,1000,clean.data(),clean.size()},cap));
  EXPECT_EQ(result.startup_payload_bytes(),cap.max_host_bytes);
  EXPECT_LT(result.owned_payload_bytes(),result.startup_payload_bytes());
  fe::ElementMassContributions tiny;
  auto underflow=clean; underflow.back().mass_source=std::numeric_limits<double>::denorm_min();
  EXPECT_FALSE(tiny.Initialize(domain,{1,.5,underflow.data(),underflow.size()}));
  EXPECT_FALSE(tiny.Initialize(domain,{2,1000,clean.data(),clean.size()}));
  EXPECT_FALSE(tiny.Initialize(domain,{1,0,clean.data(),clean.size()}));
}
TEST(NodalCoefficientElementMass, SingleArenaNamedOrderSourcePartitionsAndLegacyValues) {
  const Fixture f;
  const auto domain=f.Domain(); const auto map=f.Map(domain);
  const auto springs=f.Springs(); const auto beams=f.Contributions(domain);
  const fe::NodalCoefficientSources sources{&map,&springs,&beams};
  const auto points=Points(domain,Rows(f));
  fe::NodalCoefficientLedger old,expanded,absent;
  ASSERT_TRUE(old.Initialize(sources));
  ASSERT_TRUE(absent.InitializeWithElementMass({sources,nullptr}));
  Exact(old,absent);
  EXPECT_FALSE(old.Matches(absent));
  EXPECT_FALSE(absent.Matches(sources));
  ASSERT_TRUE(expanded.InitializeWithElementMass({sources,&points}));
  EXPECT_EQ(old.order(),fe::CoefficientOrder::PreparedSI_Q_T_B_Type25_Type13_V1);
  EXPECT_EQ(expanded.order(),fe::CoefficientOrder::PreparedSI_Q_T_B_Type25_Type13_ElementMass_V2);
  EXPECT_TRUE(expanded.MatchesWithElementMass({sources,&points}));
  EXPECT_EQ(expanded.scope().element_mass_records,3u);
  EXPECT_EQ(expanded.scope().occurrences.element_mass,3u);
  EXPECT_EQ(expanded.scope().uncovered_nodes,0u);
  for(std::size_t n=0;n<domain.node_count();++n) {
    auto expected=Values(old.nodes()[n].coefficients);
    for(const auto& point:points.records()) if(point.source.domain_node==n) {
      expected[0]+=point.mass_kg; expected[11]+=point.mass_kg;
    }
    const auto actual=Values(expanded.nodes()[n].coefficients);
    for(unsigned c=0;c<actual.size();++c) EXPECT_EQ(Bits(actual[c]),Bits(expected[c]));
  }
  EXPECT_EQ(Bits(expanded.nodes()[2].coefficients.isotropic_inertia),Bits(0.0));
  EXPECT_GT(expanded.nodes()[2].coefficients.mass,0);
}
TEST(NodalCoefficientElementMass, StructuralNamespaceLateOverflowAndDomainBackingBudget) {
  Fixture f;
  const auto domain=f.Domain(); const auto map=f.Map(domain);
  const auto beams=f.Contributions(domain); const auto springs=f.Springs();
  const fe::NodalCoefficientSources sources{&map,&springs,&beams};
  auto clean=Rows(f);
  const auto points=Points(domain,clean);
  fe::NodalCoefficientLedger expected;
  ASSERT_TRUE(expected.InitializeWithElementMass({sources,&points}));
  for(const auto id:{f.shells.qeph_source_id(0),f.shells.t3_source_id(0),
                     f.shells.qbat_source_id(0),beams.model()->connections()[0].source_id}) {
    auto bad=clean; bad.back().source_element_id=id;
    const auto duplicate=Points(domain,bad);
    fe::NodalCoefficientLedger result; const auto before=Bytes(result);
    const auto report=result.InitializeWithElementMass({sources,&duplicate});
    EXPECT_EQ(report.status,S::DuplicateIdentity); EXPECT_EQ(report.producer,fe::CoefficientProducer::ElementMass);
    EXPECT_EQ(report.parent,2u); EXPECT_EQ(Bytes(result),before);
    ASSERT_TRUE(result.InitializeWithElementMass({sources,&points})); Exact(result,expected);
  }
  // WID remains separate from the structural/point-mass EID namespace.
  clean.front().source_element_id=f.spring_input[0].source_element_id;
  // This fixture WID deliberately aliases a shell EID, so use the other WID.
  f.spring_input[0].source_element_id=990000;
  clean.front().source_element_id=990000;
  const auto distinct=f.Springs(); const auto separate_namespace=Points(domain,clean);
  fe::NodalCoefficientLedger allowed;
  ASSERT_TRUE(allowed.InitializeWithElementMass({{&map,&distinct,&beams},&separate_namespace}));

  const auto independent_domain=f.Domain();
  const auto independent_points=Points(independent_domain,Rows(f));
  fe::NodalCoefficientLedger independent;
  ASSERT_TRUE(independent.InitializeWithElementMass({sources,&independent_points}));
  EXPECT_TRUE(independent.Matches(expected));
  EXPECT_EQ(independent.owned_payload_bytes()-expected.owned_payload_bytes(),
            domain.owned_payload_bytes()-sizeof(fe::NodalNodeDomain));
  const auto independent_beams=f.Contributions(independent_domain);
  fe::NodalCoefficientLedger shared_extra_backing;
  ASSERT_TRUE(shared_extra_backing.InitializeWithElementMass(
      {{&map,&springs,&independent_beams},&independent_points}));
  EXPECT_EQ(shared_extra_backing.owned_payload_bytes(),independent.owned_payload_bytes());
  EXPECT_TRUE(shared_extra_backing.Matches(expected));
  const auto foreign_points=Points(f.Domain(2),Rows(f));
  fe::NodalCoefficientLedger foreign;
  const auto association=foreign.InitializeWithElementMass({sources,&foreign_points});
  EXPECT_EQ(association.status,S::IdentityMismatch);
  EXPECT_EQ(association.producer,fe::CoefficientProducer::ElementMass);
  EXPECT_FALSE(foreign.prepared());
  fe::NodalCoefficientLedger limited;
  auto cap=fe::CoefficientLimits{}; cap.max_host_bytes=expected.startup_payload_bytes()-1;
  EXPECT_EQ(limited.InitializeWithElementMass({sources,&points},cap).status,S::ResourceLimit);
  ++cap.max_host_bytes;
  ASSERT_TRUE(limited.InitializeWithElementMass({sources,&points},cap));
  Exact(limited,expected);

  auto huge=Rows(f); huge[0].mass_source=1e308; huge[1].mass_source=1e308;
  huge[1].domain_node=huge[0].domain_node; huge[1].source_node_id=huge[0].source_node_id;
  const auto overflow=Points(domain,huge,1);
  fe::NodalCoefficientLedger retry; const auto before=Bytes(retry);
  const auto report=retry.InitializeWithElementMass({sources,&overflow});
  EXPECT_EQ(report.status,S::NonfiniteResult); EXPECT_EQ(report.producer,fe::CoefficientProducer::ElementMass);
  EXPECT_EQ(report.parent,1u); EXPECT_EQ(Bytes(retry),before);
  ASSERT_TRUE(retry.InitializeWithElementMass({sources,&points})); Exact(retry,expected);
}
} // namespace coefficient_test
