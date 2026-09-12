// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "../extended_solid_coefficients/Fixture.h"
namespace beam18_model_test {
TEST(Beam18Ledger, V5AppendsAfterEitherPreparedSolidProfileWithoutRelaxingV4) {
  extended_solid_test::Fixture f;
  const auto domain=f.Domain(); const auto map=f.Map(domain);
  fe::SolidNodeContributions original,extended;
  const auto prior=f.Solids(domain);
  ASSERT_TRUE(extended.Initialize(domain,f.Input()));
  auto raw=beam18_force_test::Reference().input(); raw.source_element_id=999999;
  // Endpoints deliberately share actual rear and radiator nodes.
  for(unsigned k=0;k<2;++k) {
    const auto node=extended.parents()[3+k].domain_node[0];
    raw.source_node_id[k]=domain.nodes()[node].source_id;
    raw.position[k]=domain.nodes()[node].position;
  }
  // Distinct NIDs can be coincident in this synthetic fixture; choose a second
  // rear source slot with a different coordinate to form a real finite beam.
  const auto node=extended.parents()[4].domain_node[1];
  raw.source_node_id[1]=domain.nodes()[node].source_id; raw.position[1]=domain.nodes()[node].position;
  raw.source_node_id[2]=999888; raw.position[2]={2,3,4};
  b::ParentInput row; ASSERT_EQ(b::InitializeReference(raw,row.reference),b::Status::Success);
  row.material=beam18_force_test::Material(row.reference);
  b::Model model; ASSERT_TRUE(model.Initialize(domain,{1,{&row,1},b::ModelProfile::CircularFourPointLaw44V1}));
  fe::Beam18NodeContributions beam; ASSERT_TRUE(beam.Initialize(model));
  const fe::SolidNodeContributions* snapshots[]{&prior,&extended};
  for(const auto* solids:snapshots) {
    fe::NodalCoefficientLedger base,joined;
    const auto r=solids==&prior?base.InitializeWithSolids({{&map},nullptr,solids}):
      base.InitializeWithExtendedSolids({{&map},nullptr,solids});
    ASSERT_TRUE(r); ASSERT_TRUE(joined.InitializeWithBeams({{{&map},nullptr,solids},&beam}));
    for(std::size_t n=0;n<domain.node_count();++n) {
      double mass=base.nodes()[n].coefficients.mass,inertia=base.nodes()[n].coefficients.isotropic_inertia;
      for(const auto& record:beam.records()) if(record.value.global_node==n) {
        mass+=record.value.coefficients.mass_kg; inertia+=record.value.coefficients.native_total_inertia_kg_m2;
      }
      EXPECT_EQ(Bits(joined.nodes()[n].coefficients.mass),Bits(mass));
      EXPECT_EQ(Bits(joined.nodes()[n].coefficients.isotropic_inertia),Bits(inertia));
    }
    EXPECT_EQ(joined.scope().solid18_law90_parents,solids==&extended?1:0);
  }
  fe::NodalCoefficientLedger closed;
  EXPECT_EQ(closed.InitializeWithSolids({{&map},nullptr,&extended}).status,fe::CoefficientStatus::IdentityMismatch);
  EXPECT_EQ(closed.InitializeWithExtendedSolids({{&map},nullptr,&prior}).status,fe::CoefficientStatus::IdentityMismatch);
}
} // namespace beam18_model_test
