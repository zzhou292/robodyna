// SPDX-License-Identifier: MIT
#include "Fixture.h"

namespace shell_execution_test {
TEST(ShellExecutionCatalog, CompleteRolesKeepNativeGeometryAndExposeZeroRigidPoints) {
  Fixture f;
  fe::ShellSectionCounts q,t,b;
  ASSERT_TRUE(f.catalog.Counts(Family::Qeph,&q));
  ASSERT_TRUE(f.catalog.Counts(Family::T3,&t));
  ASSERT_TRUE(f.catalog.Counts(Family::Qbat,&b));
  EXPECT_EQ(q.rigid_skin,1u);
  EXPECT_EQ(t.rigid_skin,2u);
  EXPECT_EQ(q.law44,1u);
  EXPECT_EQ(t.law44,2u);
  EXPECT_EQ(t.law44_nip1,1u);
  EXPECT_EQ(b.law44_qbat,1u);
  EXPECT_EQ(b.rigid_skin,0u);
  const unsigned expected[]{4,3,0,1,3,0,0};
  for (std::size_t i = 0; i < f.catalog.parent_count(); ++i) {
    const auto& p = *f.catalog.parent(i);
    unsigned points = 999;
    ASSERT_TRUE(f.catalog.MaterialPointCount(p.family,p.family_index,&points));
    EXPECT_EQ(points,expected[i]);
  }
  fe::sections::PointParameters plastic;
  plastic.a11 = 42;
  tl::material::ShellElasticLaw1PointParameters elastic;
  elastic.elastic.a11 = 73;
  EXPECT_FALSE(f.catalog.Parameters(Family::Qeph,0,&plastic));
  EXPECT_FALSE(f.catalog.ElasticParameters(Family::Qeph,0,&elastic));
  EXPECT_EQ(plastic.a11,42);
  EXPECT_EQ(elastic.elastic.a11,73);
  unsigned unchanged = 19;
  EXPECT_FALSE(f.catalog.MaterialPointCount(Family::T3,999,&unchanged));
  EXPECT_EQ(unchanged,19u);
  qbat_binding_test::Reduction(f.shells);
  EXPECT_TRUE(f.catalog.Matches(f.shells));
  auto clone = f.catalog;
  EXPECT_TRUE(clone.SameScope(f.catalog));
  // Borrowed source edits cannot alter already prepared identity.
  f.source.base.materials[0].density_kg_m3 *= 2;
  EXPECT_TRUE(clone.SameScope(f.catalog));
}
TEST(ShellExecutionCatalog, LegacyAndMalformedDeclarationsStayClosedAndRetryable) {
  Fixture f;
  fe::ShellBatchPlasticityBinding legacy;
  EXPECT_EQ(legacy.InitializeFormulations(f.shells,f.source.Catalog()).status,Status::InvalidMaterial);
  EXPECT_FALSE(legacy.prepared());
  auto check = [&](auto mutate) {
    Source source;
    mutate(source);
    fe::ShellBatchPlasticityBinding target;
    EXPECT_NE(target.InitializeExecutionCatalog(f.shells,source.Catalog()).status,Status::Success);
    EXPECT_FALSE(target.prepared());
    EXPECT_EQ(target.InitializeExecutionCatalog(f.shells,f.source.Catalog()).status,Status::Success);
  };
  check([](Source& s) { s.base.materials[0].curve_id = 101; });
  check([](Source& s) { s.base.materials[0].linear.initial_yield_pa = -0.; });
  check([](Source& s) { s.base.materials[0].rate.enabled = true; });
  check([](Source& s) { s.base.sections[0].through_thickness_points = 3; });
  check([](Source& s) { s.base.sections[0].formulation = fe::ShellSectionFormulation::LayeredNip3; });
  check([](Source& s) { s.parents.back().family_index = 0; });
  check([](Source& s) { ++s.parents.back().source_parent_id; });
  check([](Source& s) { s.base.materials[0].density_kg_m3 = std::nextafter(2500.,3000.); });
  auto limits = fe::ShellPlasticityCatalogLimits{};
  limits.max_owned_bytes = f.catalog.host_bytes()-1;
  EXPECT_EQ(legacy.InitializeExecutionCatalog(f.shells,f.source.Catalog(),limits).status,Status::ResourceLimit);
  EXPECT_FALSE(legacy.prepared());
  limits.max_owned_bytes = f.catalog.host_bytes();
  EXPECT_EQ(legacy.InitializeExecutionCatalog(f.shells,f.source.Catalog(),limits).status,Status::Success);
}
} // namespace shell_execution_test
