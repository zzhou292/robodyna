#include "NativeOracle.h"
#include <gtest/gtest.h>

namespace rear18_test {
TEST(Rear18Native, CompleteGeometryAndActualRepeatedNodeMassScatter) {
  for (bool collapsed : {false, true}) for (bool reverse : {false, true}) {
    auto input = collapsed ? Collapsed() : Cube();
    if (reverse) for (auto& p : input.position_m) p.x = -p.x;
    law::Reference reference;
    ASSERT_EQ(law::InitializeReference(input, reference), s::Status::Success);
    const auto native = Native(input);
    Compare(reference, native);
    if (collapsed) {
      EXPECT_DOUBLE_EQ(native.node_mass[4], 2*native.node_mass[0]);
      EXPECT_DOUBLE_EQ(native.node_mass[5], 2*native.node_mass[0]);
      EXPECT_EQ(native.node_mass[6], 0);
      EXPECT_EQ(native.node_mass[7], 0);
      EXPECT_NE(native.node_mass[4], native.values[146]/6);
    }
  }
}
TEST(Rear18Native, DistortedAndTranslatedGeometryRemainsOriginalSlotOrdered) {
  for (bool collapsed : {false, true}) {
    auto input = collapsed ? Collapsed() : Cube();
    for (auto& p : input.position_m) p = {2*p.x+.1*p.y+.2*p.z+123.4, .9*p.y+.03*p.z-45.6, 1.2*p.z+8.9};
    law::Reference reference;
    ASSERT_EQ(law::InitializeReference(input, reference), s::Status::Success);
    Compare(reference, Native(input));
  }
}
}  // namespace rear18_test
