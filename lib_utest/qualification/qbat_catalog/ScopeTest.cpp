#include "Fixture.h"
#include "../nodal_mass/NodalMassTestSupport.h"

namespace qbat_catalog_test {
TEST(QbatCatalog, OptionalConnectorCompositionMustMatchEverySourceParent) {
  Fixture f;
  fe::ShellBatchBinding binding;
  ASSERT_EQ(binding.InitializeFormulations(f.Geometry()).status,fe::ShellBindingStatus::Success);
  Catalog catalog;
  ASSERT_EQ(catalog.InitializeFormulations(binding,f.Input()).status,Status::Success);
  fe::ShellBatchFailureBinding failure;
  ASSERT_EQ(failure.Initialize(catalog,f.failures.data(),6).status,Status::Success);
  const nodal_mass_test::SpringInput input(binding);
  const auto connectors=nodal_mass_test::Connectors(input,binding.node_count());
  fe::NodalMassBinding mass;
  ASSERT_TRUE(mass.Initialize(binding,connectors));
  EXPECT_EQ(fe::ValidateShellFormulationScope({&binding,&catalog,&failure,&mass}).status,Status::Success);
  // Changed original parent identity keeps identical physical M/J, but it is
  // not the immutable collection from which the optional mass was composed.
  f.geometry.b.source_parent_id=999;
  f.parents[0].source_parent_id=999;
  f.failures[0].source=f.parents[0];
  fe::ShellBatchBinding changed;
  ASSERT_EQ(changed.InitializeFormulations(f.Geometry()).status,fe::ShellBindingStatus::Success);
  qbat_binding_test::Exact(binding.totals(),changed.totals());
  Catalog changed_catalog;
  ASSERT_EQ(changed_catalog.InitializeFormulations(changed,f.Input()).status,Status::Success);
  fe::ShellBatchFailureBinding changed_failure;
  ASSERT_EQ(changed_failure.Initialize(changed_catalog,f.failures.data(),6).status,Status::Success);
  EXPECT_EQ(fe::ValidateShellFormulationScope({&changed,&changed_catalog,&changed_failure,&mass}).status,
      Status::IdentityMismatch);
  EXPECT_EQ(fe::ValidateShellFormulationScope({&changed,&catalog,&failure}).status,Status::IdentityMismatch);
  EXPECT_EQ(fe::ValidateShellFormulationScope({&changed,&changed_catalog,&changed_failure}).status,Status::Success);
}
} // namespace qbat_catalog_test
