// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
namespace extended_model_test {
TEST(ExtendedSolidModel, FiveTypedFamiliesPreserveSourceSlotsMassAndOriginalPrefix) {
  Fixture f;
  const auto domain = f.Domain();
  s::Model model, legacy;
  ASSERT_TRUE(model.Initialize(domain,f.Input()));
  ASSERT_TRUE(legacy.Initialize(domain,f.solid_model_test::Fixture::Input()));
  EXPECT_EQ(model.profile(),s::ModelProfile::ExtendedLaw44Law90);
  EXPECT_EQ(model.contributions()->profile(),fe::SolidCoefficientProfile::ExtendedLaw44Law90);
  ASSERT_EQ(model.contributions()->parents().size(),5u);
  EXPECT_EQ(model.materials36().size(),1u); EXPECT_EQ(model.materials42().size(),1u);
  EXPECT_EQ(model.materials44().size(),1u); EXPECT_EQ(model.materials90().size(),1u);
  EXPECT_EQ(model.solid24()[0].material_index,model.solid6z()[0].material_index);
  for (unsigned parent = 0; parent < 3; ++parent) {
    const auto& a = model.contributions()->parents()[parent];
    const auto& b = legacy.contributions()->parents()[parent];
    EXPECT_EQ(a.family,b.family); EXPECT_EQ(a.source_element_id,b.source_element_id);
    for (unsigned n = 0; n < a.node_count; ++n) {
      EXPECT_EQ(a.source_node_id[n],b.source_node_id[n]);
      EXPECT_EQ(a.domain_node[n],b.domain_node[n]);
      EXPECT_TRUE(Bits(a.mass_kg[n],b.mass_kg[n]));
    }
  }
  const auto& a = model.solid18_law44()[0];
  const auto& b = model.solid18_law90()[0];
  EXPECT_EQ(a.domain_nodes[4],a.domain_nodes[5]);
  EXPECT_EQ(a.domain_nodes[6],a.domain_nodes[7]);
  for (unsigned n = 0; n < 8; ++n) {
    EXPECT_EQ(a.domain_nodes[n],domain.Find(a.reference.input().source_node_id[n]));
    EXPECT_EQ(b.domain_nodes[n],domain.Find(b.reference.input().source_node_id[n]));
    EXPECT_TRUE(Bits(model.contributions()->parents()[3].mass_kg[n],f.input44[0].reference.mass().source_nodal_mass_kg[n]));
    EXPECT_TRUE(Bits(model.contributions()->parents()[4].mass_kg[n],f.input90[0].reference.mass().source_nodal_mass_kg[n]));
  }
  EXPECT_EQ(model.solid6z()[0].domain_nodes[6],SIZE_MAX);
  EXPECT_TRUE(Bits(model.contributions()->parents()[4].isotropic_inertia_kg_m2(),0.0));
  EXPECT_TRUE(Bits(model.materials90()[0].value.reader().reference_density_kg_m3,772));
}
TEST(ExtendedSolidModel, CurvesAreOwnedDeduplicatedAndSurviveAllBorrowedInputs) {
  std::unique_ptr<s::Model> model;
  {
    Fixture f; f.Repeat44(); f.Repeat90();
    model = std::make_unique<s::Model>();
    ASSERT_TRUE(model->Initialize(f.Domain(),f.Input()));
    ASSERT_EQ(model->materials44().size(),1u); ASSERT_EQ(model->materials90().size(),1u);
    EXPECT_EQ(model->solid18_law44()[0].material_index,model->solid18_law44()[1].material_index);
    EXPECT_EQ(model->solid18_law90()[0].material_index,model->solid18_law90()[1].material_index);
    EXPECT_NE(model->materials44()[0].value.curve.plastic_strain,f.rear_x);
    EXPECT_NE(model->materials90()[0].value.curve().compression_strain,f.foam_x);
    f.rear_y[2] = 8; f.foam_y[2] = 9;
  }
  const auto copy = *model; model.reset();
  EXPECT_TRUE(Bits(copy.materials44()[0].value.curve.yield_stress_pa[2],450e6));
  EXPECT_TRUE(Bits(copy.materials90()[0].value.curve().stress_pa[2],50e6));
  Fixture f; f.Repeat44(); f.Repeat90();
  s::Model independent;
  ASSERT_TRUE(independent.Initialize(f.Domain(),f.Input()));
  EXPECT_TRUE(copy.Matches(independent)); EXPECT_FALSE(copy.SharesStorage(independent));
  const auto shared = copy;
  EXPECT_TRUE(shared.SharesStorage(copy));
}
TEST(ExtendedSolidModel, PreparedUnitsAndSignedZeroCurvesRemainMaterialIdentity) {
  Fixture a,b;
  const auto params = b.input44[0].material;
  auto material = params.material; material.native_units = law44::WorkingUnits::SI;
  ASSERT_EQ(law44::Prepare(material,params.curve,b.input44[0].material),law44::Status::Ok);
  s::Model first,second;
  ASSERT_TRUE(first.Initialize(a.Domain(),a.Input()));
  ASSERT_TRUE(second.Initialize(b.Domain(),b.Input()));
  EXPECT_TRUE(first.contributions()->Matches(*second.contributions()));
  EXPECT_FALSE(first.Matches(second));
  Fixture zero; zero.foam_x[0] = -0.0; zero.PrepareFoam();
  s::Model signed_zero;
  ASSERT_TRUE(signed_zero.Initialize(zero.Domain(),zero.Input()));
  EXPECT_TRUE(std::signbit(signed_zero.materials90()[0].value.curve().compression_strain[0]));
  EXPECT_FALSE(first.Matches(signed_zero));
}
} // namespace extended_model_test
