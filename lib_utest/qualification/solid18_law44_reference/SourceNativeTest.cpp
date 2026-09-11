#include "NativeOracle.h"
#include "OriginalFixture.h"
#include <gtest/gtest.h>
#include <limits>

namespace rear18_test {
TEST(Rear18NativeOriginal, EveryOriginalCellGeometrySlotMassAndWorkingUnits) {
  unsigned repeated = 0;
  for (unsigned i = 0; i < 306; ++i) {
    const auto input = original::Input(i);
    SCOPED_TRACE(input.source_element_id);
    law::Reference reference;
    ASSERT_EQ(law::InitializeReference(input, reference), s::Status::Success);
    repeated += reference.topology() == law::SourceTopology::RepeatedPairs56And78;
    Compare(reference, Native(input));
    const auto native = Native(original::Input(i, true));
    ASSERT_EQ(native.status, 0);
    double coordinate_scale = 0;
    for (auto p : input.position_m)
      coordinate_scale = std::max({coordinate_scale, std::abs(p.x), std::abs(p.y), std::abs(p.z)});
    const double conditioning = coordinate_scale/reference.geometry().characteristic_length_m;
    EXPECT_TRUE(solid18_test::AgreeWorkingUnits(Values(reference),
                solid18_test::NativeWorkingToSI(native.values), conditioning));
    const auto mass = Scatter(reference);
    for (unsigned n = 0; n < 8; ++n) {
      EXPECT_EQ(reference.source_slot(n), static_cast<unsigned>(native.permutation[n]));
      EXPECT_TRUE(solid18_test::Close(reference.mass().source_nodal_mass_kg[reference.source_slot(n)],
                  native.native_slot_mass[n]*1000,
                  256*std::numeric_limits<double>::epsilon()*std::max(1.0, conditioning)));
      EXPECT_TRUE(solid18_test::Close(mass[n], native.node_mass[n]*1000,
                  256*std::numeric_limits<double>::epsilon()*std::max(1.0, conditioning)));
      EXPECT_EQ(mass[n] == 0, native.node_mass[n] == 0);
    }
  }
  EXPECT_EQ(repeated, 109u);
}
}  // namespace rear18_test
