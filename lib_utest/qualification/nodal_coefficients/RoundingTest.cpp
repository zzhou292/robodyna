#include "SolidFixture.h"

namespace coefficient_test {
namespace {
// Compiler binary128 arithmetic, supplied by the existing GNU host toolchain;
// no production dependency or external high-precision library is introduced.
static_assert(__FLT128_MANT_DIG__==113);
using High=__float128;
High Absolute(High value) {return value<0?-value:value;}
struct Sum {
  High exact=0;
  std::size_t count=0;
  void Add(double value) {exact+=High(value); ++count;}
  High Bound() const {
    const High nu=High(count)/(std::uint64_t{1}<<53);
    // Positive addition gamma_n bound plus conservative gradual-underflow
    // allowance. Factor two also covers the binary128 oracle reduction error
    // (unit roundoff 2^-113 versus 2^-53) for these bounded positive sums.
    return 2*(nu*exact+High(count)*High(std::numeric_limits<double>::denorm_min()))/(1-nu);
  }
  void Check(double actual) const {
    EXPECT_TRUE(Absolute(High(actual)-exact)<=Bound())
      <<"difference="<<static_cast<double>(Absolute(High(actual)-exact))
      <<" bound="<<static_cast<double>(Bound());
  }
};
using Channels=std::array<Sum,ValueCount>;
std::vector<Channels> IndependentTerms(const fe::NodalCoefficientLedger& ledger) {
  std::vector<Channels> result(ledger.nodes().size());
  const auto& map=*ledger.shells();
  const auto& binding=*map.shells();
  auto shell=[&](const auto& reference,const auto& indices) {
    for(unsigned n=0;n<indices.size();++n) {
      auto& out=result[map.owner_index(indices[n])];
      out[0].Add(reference.nodal_mass[n]); out[1].Add(reference.isotropic_inertia[n]);
      out[2].Add(reference.nodal_mass[n]); out[3].Add(reference.isotropic_inertia[n]);
      out[4].Add(reference.physical_inertia[n]); out[5].Add(reference.added_inertia[n]);
    }
  };
  for(std::size_t e=0;e<binding.qeph_count();++e) shell(binding.qeph_reference(e),binding.qeph_nodes(e));
  for(std::size_t e=0;e<binding.t3_count();++e) shell(binding.t3_reference(e),binding.t3_nodes(e));
  for(std::size_t e=0;e<binding.qbat_count();++e) shell(binding.qbat_reference(e).quadrilateral(),binding.qbat_nodes(e));
  if(ledger.type25()) for(std::size_t e=0;e<2*ledger.type25()->connection_count();++e) {
    const auto& term=ledger.type25()->endpoint_mass()[e];
    auto& out=result[term.global_node];
    out[0].Add(term.mass_kg); out[1].Add(term.isotropic_inertia_kg_m2);
    out[6].Add(term.mass_kg); out[7].Add(term.isotropic_inertia_kg_m2);
  }
  if(ledger.type13()) for(const auto& record:ledger.type13()->records()) {
    const auto& term=record.value.coefficients;
    auto& out=result[record.value.global_node];
    out[0].Add(term.mass_kg); out[1].Add(term.isotropic_inertia_kg_m2);
    out[8].Add(term.mass_kg); out[9].Add(term.isotropic_inertia_kg_m2);
    out[10].Add(term.added_inertia_kg_m2);
  }
  if(ledger.element_mass()) for(const auto& record:ledger.element_mass()->records()) {
    auto& out=result[record.source.domain_node];
    out[0].Add(record.mass_kg); out[11].Add(record.mass_kg);
  }
  if(ledger.solids()) for(const auto& parent:ledger.solids()->parents()) {
    for(unsigned k=0;k<parent.node_count;++k) {
      auto& out=result[parent.domain_node[k]];
      out[0].Add(parent.mass_kg[k]);
      out[12+static_cast<unsigned>(parent.family)].Add(parent.mass_kg[k]);
    }
  }
  return result;
}
void CheckTwoStages(const fe::NodalCoefficientLedger& ledger) {
  const auto terms=IndependentTerms(ledger);
  Channels rounded_nodes;
  std::array<High,ValueCount> true_total{},node_error_bound{};
  for(std::size_t n=0;n<terms.size();++n) {
    const auto actual=Values(ledger.nodes()[n].coefficients);
    for(unsigned c=0;c<actual.size();++c) {
      SCOPED_TRACE(c);
      SCOPED_TRACE(n);
      terms[n][c].Check(actual[c]);
      rounded_nodes[c].Add(actual[c]);
      true_total[c]+=terms[n][c].exact;
      node_error_bound[c]+=terms[n][c].Bound();
    }
  }
  const auto totals=Values(ledger.totals());
  for(unsigned c=0;c<totals.size();++c) {
    rounded_nodes[c].Check(totals[c]);
    EXPECT_TRUE(Absolute(High(totals[c])-true_total[c])<=node_error_bound[c]+rounded_nodes[c].Bound());
  }
}
}
TEST(NodalCoefficientRounding, IndependentHighPrecisionBothStagesAndOrderSensitiveSources) {
  Fixture f;
  f.spring_property.property.mass_kg=1e14;
  f.spring_property.property.isotropic_inertia_kg_m2=1e10;
  const auto d=f.Domain(); const auto map=f.Map(d);
  const auto springs=f.Springs(); const auto beams=f.Contributions(d);
  fe::NodalCoefficientLedger ledger;
  ASSERT_TRUE(ledger.Initialize({&map,&springs,&beams}));
  CheckTwoStages(ledger);
  const fe::ElementMassSource points[]={{9100,d.nodes()[f.map[0]].source_id,f.map[0],.01},
      {9101,d.nodes()[f.map[0]].source_id,f.map[0],.01},{9102,d.nodes()[2].source_id,2,1e-290}};
  fe::ElementMassContributions masses;
  ASSERT_TRUE(masses.Initialize(d,{1,1,points,3}));
  fe::NodalCoefficientLedger expanded;
  ASSERT_TRUE(expanded.InitializeWithElementMass({{&map,&springs,&beams},&masses}));
  CheckTwoStages(expanded);
  // A reversed complete producer order must actually change a represented sum,
  // proving this fixture can catch accidental subtotal reassociation.
  bool distinguished=false;
  for(std::size_t n=0;n<d.node_count();++n) {
    double reverse=0;
    for(const auto& r:beams.records()) if(r.value.global_node==n) reverse+=r.value.coefficients.mass_kg;
    for(std::size_t e=0;e<4;++e) {
      const auto& r=springs.endpoint_mass()[e];
      if(r.global_node==n) reverse+=r.mass_kg;
    }
    for(std::size_t local=0;local<f.shells.node_count();++local)
      if(f.map[local]==n) reverse+=f.shells.nodes()[local].native.mass;
    distinguished=distinguished||Bits(reverse)!=Bits(ledger.nodes()[n].coefficients.mass);
  }
  EXPECT_TRUE(distinguished);
}
TEST(NodalCoefficientRounding, MixedSolidSharedNodeTermsKeepBothReductionBounds) {
  SolidFixture f;
  f.spring_property.property.mass_kg=1e14;
  const auto domain=f.Domain(); const auto map=f.Map(domain);
  const auto springs=f.Springs(); const auto beams=f.Contributions(domain);
  const auto solids=f.Solids(domain);
  const fe::ElementMassSource point{9800,domain.nodes()[f.map[0]].source_id,f.map[0],.013};
  fe::ElementMassContributions mass;
  ASSERT_TRUE(mass.Initialize(domain,{1,1,&point,1}));
  fe::NodalCoefficientLedger ledger;
  ASSERT_TRUE(ledger.InitializeWithSolids({{&map,&springs,&beams},&mass,&solids}));
  CheckTwoStages(ledger);
}

TEST(NodalCoefficientRounding, QualifiedTinyInertiaAndLateGlobalOverflowRetry) {
  Fixture f;
  f.spring_property.property.isotropic_inertia_kg_m2=1e-300;
  const auto d=f.Domain(); const auto map=f.Map(d);
  const auto tiny=f.Springs(); const auto beams=f.Contributions(d);
  fe::NodalCoefficientLedger ledger;
  ASSERT_TRUE(ledger.Initialize({&map,&tiny,&beams}));
  CheckTwoStages(ledger);
  f.spring_property.property.mass_kg=1e308;
  const auto huge=f.Springs();
  fe::NodalCoefficientLedger retry; const auto before=Bytes(retry);
  const auto report=retry.Initialize({&map,&huge});
  EXPECT_EQ(report.status,S::NonfiniteResult);
  EXPECT_EQ(report.producer,fe::CoefficientProducer::NodeTotals);
  EXPECT_EQ(Bytes(retry),before);
  ASSERT_TRUE(retry.Initialize({&map,&tiny,&beams})); Exact(retry,ledger);
}
} // namespace coefficient_test
