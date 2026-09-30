#pragma once
#include <gtest/gtest.h>
#include <cstdint>
#include <cstring>

namespace tl::fea::seal_test {
inline std::uint64_t Bits(double value) {
  std::uint64_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}
template<class A, class B>
void SameLimit(const A& a, const B& b) {
  EXPECT_EQ(Bits(a.dt), Bits(b.dt));
  EXPECT_EQ(Bits(a.stiffness_bound), Bits(b.stiffness_bound));
  EXPECT_EQ(Bits(a.damping_bound), Bits(b.damping_bound));
  EXPECT_EQ(a.stiffness_node, b.stiffness_node);
  EXPECT_EQ(a.damping_node, b.damping_node);
  EXPECT_EQ(a.base_epoch, b.base_epoch);
  EXPECT_EQ(a.attempt, b.attempt);
  EXPECT_EQ(a.has_stiffness_or_damping, b.has_stiffness_or_damping);
}
template<class A, class B>
void SameRows(const A& a, const B& b) {
  EXPECT_EQ(a.node_count, b.node_count);
  EXPECT_EQ(a.capacity, b.capacity);
  EXPECT_EQ(a.base_epoch, b.base_epoch);
  EXPECT_EQ(a.attempt, b.attempt);
  EXPECT_EQ(a.initialized, b.initialized);
  EXPECT_EQ(a.valid, b.valid);
  EXPECT_EQ(a.sealed, b.sealed);
}
} // namespace tl::fea::seal_test
