// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NativeOracle.h"
#include "lib_src/math/ScalarBits.h"
#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>
namespace type25_assembly_test {
inline ass::Controls Controls() { return {0,0,0,0,0,{0,0,0}}; }
inline rd::NativeFrictionResult Response(double x = 12.) {
  rd::NativeFrictionResult value;
  value.contact_active = true;
  value.normal.weights[0] = .1; value.normal.weights[1] = .2;
  value.normal.weights[2] = .3; value.normal.weights[3] = .4;
  value.normal.stability_stiffness = 17.;
  value.native_resultant = {x, -3.7, 1e-10};
  return value;
}
struct Case {
  std::vector<ass::Connectivity> rows;
  std::vector<rd::NativeFrictionResult> responses;
  std::vector<std::uint32_t> cohorts;
  std::vector<ass::NativeNodalValue> incoming;
};
inline Case Corpus() {
  Case c;
  constexpr unsigned Rows = 257, Nodes = 93;
  c.rows.resize(Rows); c.responses.resize(Rows); c.incoming.resize(Nodes);
  c.cohorts = {1,2,18,129,Rows};
  for (unsigned node = 0; node < Nodes; ++node)
    c.incoming[node] = {{node % 2 ? -0. : 1e20, -1.25 * node, 2.75}, .0625 * node};
  for (unsigned row = 0; row < Rows; ++row) {
    c.rows[row] = {{row%Nodes,(3*row+7)%Nodes,(5*row+19)%Nodes,(5*row+19)%Nodes},(11*row+3)%Nodes};
    auto& r = c.responses[row];
    r = Response(row % 3 == 0 ? -1e20 : (row % 3 == 1 ? 1e20 : .03125*row));
    r.normal.stability_stiffness = row % 11 == 0 ? -0. : row * .17;
    r.normal.weights[0] = row % 7 == 0 ? -0. : -.2;
    r.normal.weights[1] = row % 7 == 0 ? 0. : .3;
    r.normal.weights[2] = row % 7 == 0 ? .5 : .4;
    r.normal.weights[3] = .5;
    if (row % 13 == 0) {
      r.contact_active = false;
      for (double& weight : r.normal.weights) weight = 0.;
      r.native_resultant = {std::numeric_limits<double>::quiet_NaN(),0,0};
    }
    if (r.contact_active && row % 17 == 1) { // Exact HH-zero: skip all slots.
      r.normal.weights[0] = 1.; r.normal.weights[1] = -1.;
      r.normal.weights[2] = -0.; r.normal.weights[3] = 0.;
    }
  }
  return c;
}
inline void Same(const ass::NativeNodalValue& a, const ass::NativeNodalValue& b) {
  EXPECT_TRUE(tl::math::SameScalarBits(a.force.x,b.force.x)) << a.force.x << " vs " << b.force.x;
  EXPECT_TRUE(tl::math::SameScalarBits(a.force.y,b.force.y));
  EXPECT_TRUE(tl::math::SameScalarBits(a.force.z,b.force.z));
  EXPECT_TRUE(tl::math::SameScalarBits(a.stiffness,b.stiffness));
}
inline std::vector<ass::NativeNodalValue> Gather(const Case& c) {
  const ass::Schedule schedule{c.cohorts.empty() ? nullptr : c.cohorts.data(),c.cohorts.size(),c.rows.size()};
  std::vector<std::uint32_t> offsets(c.incoming.size()+1), ranks(5*c.rows.size());
  if (!ass::BuildIncidence(c.rows.empty() ? nullptr : c.rows.data(),schedule,c.incoming.size(),
      offsets.data(),offsets.size(),ranks.empty() ? nullptr : ranks.data(),ranks.size()))
    throw std::runtime_error("Test incidence setup failed");
  std::vector<ass::NativeEndpoints> packets(c.rows.size());
  for (std::size_t row = 0; row < packets.size(); ++row)
    if (ass::PrepareNativeEndpoints(Controls(),c.responses[row],&packets[row]) != ass::Status::Ok)
      throw std::runtime_error("Test endpoint preparation failed");
  auto result = c.incoming;
  for (std::size_t node = 0; node < result.size(); ++node)
    if (ass::GatherNode(node,c.rows.data(),packets.data(),schedule,
        {offsets.data(),ranks.data(),result.size(),ranks.size()},c.incoming[node],&result[node]) != ass::Status::Ok)
      throw std::runtime_error("Test node gather failed");
  return result;
}
} // namespace type25_assembly_test
