#pragma once
#include "Fixture.h"
#include <algorithm>
namespace classification_test {
inline Fixture ProjectedFixture() {
  Fixture f;
  f.roles = {{100,2,28,{0,1,2},{6,7,8,9,10,11}}};
  f.groups = {{6,7,8},{9,7,10},{10,11}};
  return f;
}
inline void SameObserved(const ClassificationResult& projected, const ClassificationResult& complete) {
  ASSERT_EQ(projected.slave_nodes().count, complete.slave_nodes().count);
  for (std::size_t row = 0; row < projected.slave_nodes().count; ++row) {
    const auto node = projected.slave_nodes().data[row];
    ASSERT_EQ(node, complete.slave_nodes().data[row]);
    EXPECT_EQ(projected.irupt().data[row], complete.irupt().data[row]);
    SameKinematics(projected.nodes().data[node].kinematics, complete.nodes().data[node].kinematics);
  }
  ASSERT_EQ(projected.interface_decode().count, 8192u);
  ASSERT_EQ(complete.interface_decode().count, 8192u);
  for (std::size_t word = 0; word < 8192; ++word)
    EXPECT_EQ(projected.interface_decode().data[word], complete.interface_decode().data[word]);
  EXPECT_EQ(projected.native_penalty_warnings(), complete.native_penalty_warnings());
  EXPECT_EQ(projected.native_kinset_warnings(), complete.native_kinset_warnings());
}
}
