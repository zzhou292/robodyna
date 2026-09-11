// SPDX-License-Identifier: MIT
#include "Fixture.h"
#include <memory>

namespace shell_physical_test {
TEST(ShellPhysicalBinding, MappedLayeredShellsRetainWholeCoefficientsAndExactNativeRoles) {
  Fixture f;
  fe::ShellPhysicalBinding binding;
  ASSERT_TRUE(binding.Initialize(f.Scope(),f.ledger));
  EXPECT_FALSE(binding.mapping()->identity_map());
  EXPECT_EQ(binding.domain()->node_count(),f.shells.node_count()+1);
  EXPECT_EQ(binding.shells()->qbat_count(),1);
  EXPECT_EQ(binding.shells()->t3_count(),3);
  EXPECT_EQ(binding.shells()->qeph_count(),2);
  EXPECT_EQ(binding.coefficients()->nodes()[0].coefficients.mass,2);
  EXPECT_EQ(binding.coefficients()->nodes()[0].coefficients.isotropic_inertia,0);
  for (std::size_t i = 0; i < f.shells.node_count(); ++i) {
    const auto owner = binding.mapping()->owner_index(i);
    EXPECT_EQ(owner,f.shells.node_count()-i);
    EXPECT_EQ(binding.domain()->nodes()[owner].source_id,f.shells.nodes()[i].source_id);
    EXPECT_DOUBLE_EQ(binding.coefficients()->nodes()[owner].coefficients.mass,
        f.shells.nodes()[i].native.mass);
  }
  qbat_catalog_test::CheckMidlayer(*binding.catalog(),fe::ShellBindingFamily::Qbat,0,
      fe::ShellSectionLaw::Law44QbatFourInPlane);
  EXPECT_TRUE(binding.failure()->SameScope(f.failure));
}
TEST(ShellPhysicalBinding, CoverageAndForeignCatalogRejectWithoutPublishingThenRetry) {
  Fixture f,missing(false);
  fe::ShellPhysicalBinding binding;
  EXPECT_EQ(binding.Initialize(missing.Scope(),missing.ledger).status,fe::NodalDomainStatus::MissingSource);
  EXPECT_FALSE(binding.prepared());
  f.source.y[2] *= 1.01;
  fe::ShellBatchPlasticityBinding foreign;
  ASSERT_EQ(foreign.InitializeFormulations(f.shells,f.source.Input()).status,
      fe::ShellPlasticityBindingStatus::Success);
  auto wrong = f.Scope();
  wrong.catalog = &foreign;
  EXPECT_FALSE(binding.Initialize(wrong,f.ledger));
  EXPECT_FALSE(binding.prepared());
  EXPECT_TRUE(binding.Initialize(f.Scope(),f.ledger));
  EXPECT_EQ(binding.Initialize(f.Scope(),f.ledger).status,fe::NodalDomainStatus::AlreadyInitialized);
}
TEST(ShellPhysicalBinding, ExactRetainedBudgetRejectsOneByteShortAndRetries) {
  Fixture f;
  fe::ShellPhysicalBinding first;
  ASSERT_TRUE(first.Initialize(f.Scope(),f.ledger));
  fe::ShellPhysicalBinding second;
  fe::ShellPhysicalBindingLimits limits;
  limits.max_host_bytes = first.owned_payload_bytes()-1;
  EXPECT_EQ(second.Initialize(f.Scope(),f.ledger,limits).status,fe::NodalDomainStatus::ResourceLimit);
  EXPECT_FALSE(second.prepared());
  ++limits.max_host_bytes;
  ASSERT_TRUE(second.Initialize(f.Scope(),f.ledger,limits));
  EXPECT_TRUE(first.Matches(second));
  EXPECT_EQ(first.owned_payload_bytes(),second.owned_payload_bytes());
}
TEST(ShellPhysicalBinding, SourceLifetimeAndIndependentLedgerIdentityArePreserved) {
  fe::ShellPhysicalBinding binding;
  {
    Fixture f;
    ASSERT_TRUE(binding.Initialize(f.Scope(),f.ledger));
  }
  const auto copy = binding;
  EXPECT_TRUE(copy.Matches(binding));
  EXPECT_EQ(copy.domain()->nodes()[0].source_id,900001);
  fe::sections::PointParameters material;
  ASSERT_TRUE(copy.catalog()->Parameters(fe::ShellBindingFamily::Qeph,0,&material));
  ASSERT_EQ(material.curve.count,3);
  EXPECT_EQ(material.curve.yield_stress_pa[2],20e6);
  Fixture changed(true,3);
  fe::ShellPhysicalBinding other;
  ASSERT_TRUE(other.Initialize(changed.Scope(),changed.ledger));
  EXPECT_FALSE(copy.Matches(other));
  // A declared zero point mass still has a producer. DOF admission is owned by
  // the physical owner, so this source binding never substitutes a mass floor.
  Fixture zero(true,0);
  fe::ShellPhysicalBinding zero_binding;
  ASSERT_TRUE(zero_binding.Initialize(zero.Scope(),zero.ledger));
  EXPECT_EQ(zero_binding.coefficients()->nodes()[0].coefficients.mass,0);
}
} // namespace shell_physical_test
