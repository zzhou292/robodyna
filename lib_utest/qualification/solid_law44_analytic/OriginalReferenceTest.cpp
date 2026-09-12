// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
#include "AirbagFixture.h"
#include "lib_utest/qualification/solid18_law44_reference/NativeOracle.h"
#include "lib_src/elements/solid18/law44/Force.h"

namespace law44_analytic_test {
TEST(SolidLaw44AnalyticOriginal, All80SelectedReferencesNativeMassAndFourCollapsedSlots) {
  unsigned repeated = 0;
  for (unsigned i = 0; i < 80; ++i) {
    const auto input = law44_airbag_fixture::Input(i);
    SCOPED_TRACE(input.source_element_id);
    tl::fea::solid18::law44::Reference reference;
    ASSERT_EQ(tl::fea::solid18::law44::InitializeReference(input, reference), tl::fea::solid18::Status::Success);
    repeated += reference.topology() == tl::fea::solid18::law44::SourceTopology::RepeatedPairs56And78;
    rear18_test::Compare(reference, rear18_test::Native(input));
    const auto native = rear18_test::Native(law44_airbag_fixture::Input(i, true));
    ASSERT_EQ(native.status, 0);
    double coordinate_scale = 0;
    for (auto x : input.position_m)
      coordinate_scale = std::max({coordinate_scale, std::abs(x.x), std::abs(x.y), std::abs(x.z)});
    const double conditioning = coordinate_scale / reference.geometry().characteristic_length_m;
    EXPECT_TRUE(solid18_test::AgreeWorkingUnits(rear18_test::Values(reference),
        solid18_test::NativeWorkingToSI(native.values), conditioning));
    for (unsigned n = 0; n < 8; ++n) {
      EXPECT_GT(reference.geometry().point[n].initial_volume_m3, 0);
      EXPECT_GT(reference.mass().source_nodal_mass_kg[n], 0);
      EXPECT_EQ(reference.source_slot(n), static_cast<unsigned>(native.permutation[n]));
    }
    tl::fea::solid18::law44::ForceTrial initial;
    ASSERT_EQ(tl::fea::solid18::law44::InitializeForce(reference, Airbag(), {}, initial),
        tl::fea::solid18::Status::Success);
    for (const auto& point : initial.proposed_history.data().point)
      EXPECT_EQ(point.material.curve_cursor, 0u);
  }
  EXPECT_EQ(repeated, 4u);
  RecordProperty("original_isolid", 5);
  RecordProperty("selected_demo_isolid", 18);
  RecordProperty("original_cells", 80);
  RecordProperty("original_nodes", 156);
}
} // namespace law44_analytic_test
