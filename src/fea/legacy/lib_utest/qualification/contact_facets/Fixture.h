// SPDX-License-Identifier: MIT
#pragma once
#include "../self_contact_surface/Fixture.h"
#include "lib_src/collision/FixedContactFacetBinding.h"
#include "lib_src/collision/FixedContactFacetValues.h"
#include <algorithm>

namespace facet_test {
namespace ct = tlfea::contact;
using S = ct::FixedContactFacetStatus;
struct Fixture {
  self_contact_test::Fixture source;
  ct::SelfContactSurfaceBinding surface;
  explicit Fixture(bool distinct = false) : source(true, 77, distinct) {
    const auto rows = source.Selection();
    EXPECT_EQ(surface.Initialize(source.physical, self_contact_test::Input(rows)).status,
        ct::SelfContactSurfaceStatus::Ok);
  }
  std::size_t Parent(std::uint64_t eid) const {
    for (std::size_t i = 0; i < surface.parents().size(); ++i)
      if (surface.parents()[i].source.source_parent_id == eid) return i;
    throw std::invalid_argument("Missing source EID in test fixture");
  }
  std::vector<double> Positions(std::size_t parent, const ct::Vec3* points) const {
    std::vector<double> result(3 * source.domain.node_count());
    const auto& native = surface.parents()[parent];
    for (unsigned i = 0; i < native.arity; ++i) {
      const auto node = native.arity == 4 ? native.q4.nodes[i] : native.t3.nodes[i];
      result[3 * node] = points[i].x;
      result[3 * node + 1] = points[i].y;
      result[3 * node + 2] = points[i].z;
    }
    return result;
  }
};
inline ct::VectorView View(const std::vector<double>& points) {
  return {points.data(), static_cast<std::uint32_t>(points.size() / 3), 3, 1};
}
inline ct::Vec3 Cross(ct::Vec3 a, ct::Vec3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline void Equal(ct::Vec3 a, ct::Vec3 b) {
  EXPECT_DOUBLE_EQ(a.x, b.x);
  EXPECT_DOUBLE_EQ(a.y, b.y);
  EXPECT_DOUBLE_EQ(a.z, b.z);
}
} // namespace facet_test
