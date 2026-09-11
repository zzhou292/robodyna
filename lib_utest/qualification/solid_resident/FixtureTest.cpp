// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"

namespace solid_resident_test {
TEST(SolidResidentFixture, CompleteTypedModelMatchesTheSameRigidAndCinCoefficientDomain) {
  Fixture fixture;
  ASSERT_TRUE(fixture.model.prepared());
  ASSERT_TRUE(fixture.model.contributions()->Matches(*fixture.mechanics.ledger.solids()));
  EXPECT_TRUE(fixture.model.domain()->SharesStorage(fixture.mechanics.domain));
  EXPECT_TRUE(fixture.mechanics.binding.coefficients()->Matches(fixture.mechanics.ledger));
  EXPECT_EQ(fixture.model.solid18().size(),1u);
  EXPECT_EQ(fixture.model.solid24().size(),1u);
  EXPECT_EQ(fixture.model.solid6z().size(),1u);
  EXPECT_EQ(fixture.mechanics.ledger.nodes()[fixture.mechanics.domain.Find(55)].coefficients.mass,2.);
  EXPECT_EQ(fixture.mechanics.ledger.nodes()[fixture.mechanics.domain.Find(778)].coefficients.isotropic_inertia,0);
  EXPECT_TRUE(fixture.mechanics.binding.FindMember(fixture.mechanics.domain.Find(778)));
  s::BatchForecast forecast;
  ASSERT_TRUE(s::Batch::Forecast(fixture.Configuration(),fixture.model,forecast));
}
} // namespace solid_resident_test
