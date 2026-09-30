#include "VehicleCatalogFixture.h"
#include <iostream>

namespace vehicle_catalog_test {
TEST(VehiclePlasticityCatalogSourceSize, CompleteNoTireCountsAnd875OwnedMixedDeclarations) {
  using Source=vehicle_shell_test::Fixture;
  Fixture f(Source::SourceQ,Source::SourceT,Source::SourceNodes,875);
  fe::ShellBatchBinding b;PrepareGeometry(f,b);Catalog c;
  ASSERT_EQ(c.InitializeCatalog(b,f.input(),f.limits()).status,Status::Success);
  EXPECT_EQ(c.parent_count(),349645u);EXPECT_EQ(b.node_count(),359785u);
  EXPECT_EQ(c.material_count(),875u);EXPECT_EQ(c.section_count(),875u);EXPECT_EQ(c.curve_point_count(),2u);
  EXPECT_TRUE(c.Matches(b));
  for(std::size_t i=0;i<f.parents.size();++i) CheckParent(f,c,i);
  Catalog copy(c);EXPECT_TRUE(copy.SameScope(c));
  std::cout<<"vehicle_catalog_owned_bytes="<<c.host_bytes()
           <<" vehicle_catalog_startup_scratch_bytes="<<c.startup_scratch_bytes()<<'\n';
  // A late source assignment cannot publish a partly validated vehicle catalog.
  const auto saved=f.parents.back();f.parents.back().source_parent_id=1;
  Catalog retry;const auto before=Bytes(retry);auto report=retry.InitializeCatalog(b,f.input(),f.limits());
  EXPECT_EQ(report.status,Status::IdentityMismatch);EXPECT_EQ(report.entry,f.parents.size()-1);
  EXPECT_EQ(Bytes(retry),before);f.parents.back()=saved;
  ASSERT_EQ(retry.InitializeCatalog(b,f.input(),f.limits()).status,Status::Success);
  EXPECT_TRUE(retry.SameScope(c));
}
} // namespace vehicle_catalog_test
