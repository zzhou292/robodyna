// SPDX-License-Identifier: MIT
#include "OwnerFixture.h"

namespace qt_mapped_test {
TEST(QtMappedStartup, CompleteSkinOwnerFixtureHasActualPartAndCanonicalUnusedControls) {
  Fixture fixture;
  MakeSkinFixture(fixture);
  ASSERT_TRUE(fixture.skin);
  ASSERT_TRUE(fixture.Physical().prepared());
  EXPECT_EQ(fixture.Rigid().groups().size(),1u);
  EXPECT_EQ(fixture.Rigid().members().size(),9u);
  unsigned points=99;
  EXPECT_EQ(Law(fixture.Catalog(),fe::ShellBindingFamily::Qeph,0),fe::ShellSectionLaw::RigidSkin);
  EXPECT_EQ(Law(fixture.Catalog(),fe::ShellBindingFamily::T3,0),fe::ShellSectionLaw::Law44Nip1);
  ASSERT_TRUE(fixture.Catalog().MaterialPointCount(fe::ShellBindingFamily::Qeph,0,&points));
  EXPECT_EQ(points,0u);
  ASSERT_TRUE(fixture.Catalog().MaterialPointCount(fe::ShellBindingFamily::Qeph,1,&points));
  EXPECT_EQ(points,3u);
  ASSERT_TRUE(fixture.Catalog().MaterialPointCount(fe::ShellBindingFamily::Qbat,0,&points));
  EXPECT_EQ(points,4u);
  Fixture triangle;
  MakeSkinFixture(triangle,true);
  ASSERT_TRUE(triangle.Physical().prepared());
  EXPECT_EQ(Law(triangle.Catalog(),fe::ShellBindingFamily::T3,0),fe::ShellSectionLaw::RigidSkin);
  for (std::size_t node=0;node<fixture.mechanics.domain.node_count();++node) {
    EXPECT_EQ(fixture.mechanics.m[node],fixture.ledger.nodes()[node].coefficients.mass);
    EXPECT_EQ(fixture.mechanics.j[node],fixture.ledger.nodes()[node].coefficients.isotropic_inertia);
  }
}
} // namespace qt_mapped_test
