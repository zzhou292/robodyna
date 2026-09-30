// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
#include "lib_utest/qualification/extended_solid_resident/OwnerFixture.h"
namespace law44_analytic_test {
TEST(SolidLaw44AnalyticModel, ActualMixedFixtureClosesLedgerRigidCinAndMaterialAuthority) {
  extended_resident_test::OwnerFixture fixture(true);
  ASSERT_FALSE(HasFailure());
  ASSERT_TRUE(fixture.model.prepared());
  ASSERT_EQ(fixture.model.solid18_law44().size(), 1u);
  Empty(fixture.model.materials44()[0].value);
  EXPECT_EQ(fixture.model.solid18_law44()[0].reference.input().source_material_id, 2000945u);
  EXPECT_EQ(fixture.model.materials44()[0].value.material.density_kg_m3, 1.95e-9 * 1e12);
  tl::fea::solids::BatchForecast forecast;
  ASSERT_TRUE(tl::fea::solids::Batch::Forecast(fixture.Configuration(), fixture.model, forecast));
  EXPECT_GT(forecast.device_bytes, 0u);
}
}
