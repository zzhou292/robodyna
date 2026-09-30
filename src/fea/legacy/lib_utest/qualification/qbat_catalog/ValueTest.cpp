#include "Fixture.h"

namespace qbat_catalog_test {
TEST(QbatCatalog, SharedOriginalNipOneResolvesIndependentOneAndFourPointRoles) {
  Fixture f;
  fe::ShellBatchBinding binding;
  ASSERT_EQ(binding.InitializeFormulations(f.Geometry()).status,fe::ShellBindingStatus::Success);
  qbat_binding_test::Reduction(binding);
  Catalog catalog;
  ASSERT_EQ(catalog.InitializeFormulations(binding,f.Input()).status,Status::Success);
  EXPECT_TRUE(catalog.formulation_sections());
  EXPECT_EQ(catalog.parent_count(),6u);
  CheckMidlayer(catalog,Family::Qbat,0,fe::ShellSectionLaw::Law44QbatFourInPlane);
  CheckMidlayer(catalog,Family::T3,2,fe::ShellSectionLaw::Law44Nip1);
  for(unsigned i=0;i<6;++i) {
    ASSERT_NE(catalog.parent(i),nullptr);
    EXPECT_EQ(catalog.parent(i)->source_parent_id,f.parents[i].source_parent_id);
  }
  fe::ShellSectionCounts q,t,b;
  ASSERT_TRUE(catalog.Counts(Family::Qeph,&q));
  ASSERT_TRUE(catalog.Counts(Family::T3,&t));
  ASSERT_TRUE(catalog.Counts(Family::Qbat,&b));
  EXPECT_EQ(q.law44,2u);
  EXPECT_EQ(t.law44,3u);
  EXPECT_EQ(t.law44_nip1,1u);
  EXPECT_EQ(t.law44_qbat,0u);
  EXPECT_EQ(b.law44,1u);
  EXPECT_EQ(b.law44_qbat,1u);
  EXPECT_EQ(b.law44_nip1,0u);
  fe::ShellBatchFailureBinding failure;
  ASSERT_EQ(failure.Initialize(catalog,f.failures.data(),6).status,Status::Success);
  ASSERT_NE(failure.parent(Family::Qbat,0),nullptr);
  EXPECT_EQ(failure.parent(Family::Qbat,0)->source.source_parent_id,103u);
  EXPECT_EQ(fe::ValidateShellFormulationScope({&binding,&catalog,&failure}).status,Status::Success);
  EXPECT_EQ(failure.parent(Family::Qbat,1),nullptr);
  fe::sections::PointParameters sentinel;
  sentinel.a11=123;
  const auto old=Bytes(sentinel);
  EXPECT_FALSE(catalog.Parameters(Family::Qbat,1,&sentinel));
  EXPECT_FALSE(catalog.Parameters(static_cast<Family>(77),0,&sentinel));
  EXPECT_EQ(Bytes(sentinel),old);
}

TEST(QbatCatalog, EmptyQephAndT3FamiliesNeedNoInventedRows) {
  Fixture f;
  const auto material=Midlayer();
  const auto section=MidlayerSection();
  const auto parent=f.parents[0];
  const fe::ShellBatchPlasticityBindingInput input{nullptr,&material,&section,&parent,0,1,1,1};
  fe::ShellBatchBinding binding;
  ASSERT_EQ(binding.InitializeFormulations({{nullptr,nullptr,0,0,4},&f.geometry.b,1}).status,
      fe::ShellBindingStatus::Success);
  Catalog catalog;
  ASSERT_EQ(catalog.InitializeFormulations(binding,input).status,Status::Success);
  for(auto family:{Family::Qeph,Family::T3}) {
    fe::ShellSectionCounts count;
    ASSERT_TRUE(catalog.Counts(family,&count));
    EXPECT_EQ(count.law1+count.law44,0u);
    fe::ShellSectionLaw law=fe::ShellSectionLaw::Unspecified;
    EXPECT_FALSE(catalog.Law(family,0,&law));
  }
  const auto row=Failure(parent);
  fe::ShellBatchFailureBinding failure;
  ASSERT_EQ(failure.Initialize(catalog,&row,1).status,Status::Success);
  EXPECT_EQ(fe::ValidateShellFormulationScope({&binding,&catalog,&failure}).status,Status::Success);
}

TEST(QbatCatalog, OwnedMixedCurvesAndSharedFailureScopeSurviveCallerLifetime) {
  std::unique_ptr<Catalog> copied;
  fe::ShellBatchFailureBinding failure;
  {
    Fixture f;
    fe::ShellBatchBinding binding;
    ASSERT_EQ(binding.InitializeFormulations(f.Geometry()).status,fe::ShellBindingStatus::Success);
    Catalog catalog;
    ASSERT_EQ(catalog.InitializeFormulations(binding,f.Input()).status,Status::Success);
    copied=std::make_unique<Catalog>(catalog);
    ASSERT_EQ(failure.Initialize(catalog,f.failures.data(),6).status,Status::Success);
    EXPECT_TRUE(copied->SameScope(catalog));
    f.x[2]=99;
    f.y[0]=-1;
    f.materials[2].linear.initial_yield_pa=7;
  }
  ASSERT_TRUE(failure.Matches(*copied));
  fe::sections::PointParameters p;
  ASSERT_TRUE(copied->Parameters(Family::Qeph,0,&p));
  EXPECT_DOUBLE_EQ(p.curve.plastic_strain[2],.1);
  EXPECT_DOUBLE_EQ(p.curve.yield_stress_pa[0],10e6);
  CheckMidlayer(*copied,Family::Qbat,0,fe::ShellSectionLaw::Law44QbatFourInPlane);
  CheckMidlayer(*copied,Family::T3,2,fe::ShellSectionLaw::Law44Nip1);
}
} // namespace qbat_catalog_test
