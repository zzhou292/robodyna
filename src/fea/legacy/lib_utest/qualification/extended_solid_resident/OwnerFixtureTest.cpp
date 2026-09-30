// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OwnerFixture.h"
namespace extended_resident_test {
TEST(ExtendedResidentOwnerFixture, ActualV4LedgerUsesEveryNativeSlotAndBothRigidAuthorities) {
  OwnerFixture fixture;
  const auto& f=fixture.Mechanics();
  ASSERT_TRUE(fixture.model.prepared());
  EXPECT_EQ(fixture.ledger.order(),fe::CoefficientOrder::PreparedSI_Q_T_B_Type25_Type13_ElementMass_Solid18_24_6z_Law44_Law90_V4);
  EXPECT_TRUE(fixture.model.contributions()->Matches(*fixture.ledger.solids()));
  EXPECT_TRUE(fixture.binding.coefficients()->Matches(fixture.ledger));
  EXPECT_GT(fixture.binding.groups().size(),fixture.parts.roots().size());
  EXPECT_EQ(fixture.model.solid18_law44()[0].domain_nodes[4],fixture.model.solid18_law44()[0].domain_nodes[5]);
  d::ArenaLayout layout;
  ASSERT_TRUE(d::Plan(fixture.Configuration(),fixture.model,layout));
}
} // namespace extended_resident_test
