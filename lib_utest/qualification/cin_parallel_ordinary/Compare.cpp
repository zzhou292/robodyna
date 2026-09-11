// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Packet.h"
#include <gtest/gtest.h>
#include <cstring>

namespace tl::fea::cin_parallel_test {
namespace {
void SameDouble(double a, double b) {
  EXPECT_EQ(std::memcmp(&a, &b, sizeof(double)), 0) << a << " versus " << b;
}
void SameVector(tl::math::Vec3 a, tl::math::Vec3 b) {
  SameDouble(a.x, b.x);
  SameDouble(a.y, b.y);
  SameDouble(a.z, b.z);
}
}
void SameDoubles(const std::vector<double>& a, const std::vector<double>& b) {
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i=0; i<a.size(); ++i) {
    SCOPED_TRACE(i);
    SameDouble(a[i], b[i]);
  }
}
void SameControl(const nodal_detail::Control& a, const nodal_detail::Control& b) {
  EXPECT_EQ(a.status, b.status);
  EXPECT_EQ(a.node, b.node);
  EXPECT_EQ(a.rows.node_count, b.rows.node_count);
  EXPECT_EQ(a.rows.capacity, b.rows.capacity);
  EXPECT_EQ(a.rows.base_epoch, b.rows.base_epoch);
  EXPECT_EQ(a.rows.attempt, b.rows.attempt);
  EXPECT_EQ(a.rows.initialized, b.rows.initialized);
  EXPECT_EQ(a.rows.valid, b.rows.valid);
  EXPECT_EQ(a.rows.sealed, b.rows.sealed);
  EXPECT_EQ(a.assembly.status, b.assembly.status);
  EXPECT_EQ(a.assembly.node, b.assembly.node);
  EXPECT_EQ(a.assembly.base_epoch, b.assembly.base_epoch);
  EXPECT_EQ(a.assembly.attempt, b.assembly.attempt);
  SameDouble(a.limit.dt, b.limit.dt);
  SameDouble(a.limit.stiffness_bound, b.limit.stiffness_bound);
  SameDouble(a.limit.damping_bound, b.limit.damping_bound);
  EXPECT_EQ(a.limit.stiffness_node, b.limit.stiffness_node);
  EXPECT_EQ(a.limit.damping_node, b.limit.damping_node);
  EXPECT_EQ(a.limit.base_epoch, b.limit.base_epoch);
  EXPECT_EQ(a.limit.attempt, b.limit.attempt);
  EXPECT_EQ(a.limit.has_stiffness_or_damping, b.limit.has_stiffness_or_damping);
}
void SameSuccessfulPacket(const Packet& a, const Packet& b) {
  ASSERT_EQ(a.control.status, NodalStatus::Ok);
  ASSERT_EQ(b.control.status, NodalStatus::Ok);
  SameControl(a.control, b.control);
  // Every double of state, CIN tail, loads, entry IN/A/AR, and optional capture.
  SameDoubles(a.accepted, b.accepted);
  SameDoubles(a.trial, b.trial);
  SameDoubles(a.loads, b.loads);
  SameDoubles(a.work, b.work);
  SameDoubles(a.capture, b.capture);
  ASSERT_EQ(a.patches.size(), b.patches.size());
  for (std::size_t i=0; i<a.patches.size(); ++i) {
    SCOPED_TRACE(i);
    EXPECT_EQ(a.patches[i].prepared(), b.patches[i].prepared());
    const auto& av = a.patches[i].values();
    const auto& bv = b.patches[i].values();
    SameVector(av.center, bv.center);
    SameVector(av.secondary_offset, bv.secondary_offset);
    for (unsigned s=0; s<4; ++s) SameVector(av.master_offset[s], bv.master_offset[s]);
    for (unsigned c=0; c<7; ++c) SameDouble(av.cofactor[c], bv.cofactor[c]);
  }
}
} // namespace tl::fea::cin_parallel_test
