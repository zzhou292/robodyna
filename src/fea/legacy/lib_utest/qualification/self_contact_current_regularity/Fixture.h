// SPDX-License-Identifier: MIT
#pragma once

#include "../self_contact_active_uses/Fixture.h"
#include "lib_src/collision/SelfContactCurrentRegularity.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <vector>

namespace current_regularity_test {

namespace c = tlfea::contact;
using Status = c::SelfContactCurrentRegularityStatus;

struct Fixture {
  active_use_test::Fixture source;
  c::SelfContactActiveUseBinding uses;
  c::SelfContactCurrentRegularity regularity;
  std::vector<double> positions;
  std::vector<std::uint8_t> base;
  std::vector<std::uint8_t> current;

  explicit Fixture(unsigned level = 0, bool distinct = false,
                   bool reverse_selection = false)
      : source(level, false, distinct, reverse_selection) {
    EXPECT_EQ(uses.Initialize(source.facets).status,
              c::SelfContactActiveUseStatus::Ok);
    positions.resize(3*source.domain.node_count());
    ResetReferencePositions();
    base.assign(uses.parents().size(), 1);
    current = base;
  }

  void InitializeRegularity(
      c::SelfContactCurrentRegularityLimits limits = {}) {
    EXPECT_EQ(regularity.Initialize(uses, limits).status, Status::Ok);
  }

  c::VectorView Positions() const {
    return {positions.data(),
            static_cast<std::uint32_t>(source.domain.node_count()), 3, 1};
  }

  c::SelfContactActivityView Activity() const {
    return {base.data(), current.data(), base.size()};
  }

  void ResetReferencePositions() {
    for (std::size_t n = 0; n < source.domain.node_count(); ++n) {
      const auto value = source.domain.nodes()[n].position;
      positions[3*n] = value.x;
      positions[3*n+1] = value.y;
      positions[3*n+2] = value.z;
    }
  }

  std::size_t FirstParent(unsigned arity) const {
    for (std::size_t p = 0; p < uses.parents().size(); ++p)
      if (uses.parents()[p].arity == arity) return p;
    return SIZE_MAX;
  }

  void Only(std::size_t parent, bool removing = false) {
    std::fill(base.begin(), base.end(), 0);
    std::fill(current.begin(), current.end(), 0);
    ASSERT_LT(parent, base.size());
    base[parent] = 1;
    current[parent] = removing ? 0 : 1;
  }

  template <std::size_t N>
  void SetParent(std::size_t parent,
                 const std::array<c::Vec3, N>& points) {
    ASSERT_LT(parent, uses.parents().size());
    const auto& row = uses.parents()[parent];
    ASSERT_EQ(row.arity, N);
    for (unsigned i = 0; i < row.arity; ++i) {
      const auto node = row.nodes[i];
      positions[3*node] = points[i].x;
      positions[3*node+1] = points[i].y;
      positions[3*node+2] = points[i].z;
    }
  }

  const c::SelfContactCurrentParentResult& Result(
      std::size_t parent) const {
    const auto view = regularity.results();
    EXPECT_TRUE(view.complete);
    EXPECT_LT(parent, view.count);
    return view.data[parent];
  }
};

inline const std::array<c::Vec3, 4> FlatQ4{{
    {1, 1, 0}, {-1, 1, 0}, {-1, -1, 0}, {1, -1, 0}}};
inline const std::array<c::Vec3, 4> SkewQ4{{
    {2, 1, .25}, {-.5, 1, 0}, {-1, -1, .125},
    {1.5, -1, .375}}};
inline const std::array<c::Vec3, 4> SaddleQ4{{
    {1, 1, 1}, {-1, 1, -1}, {-1, -1, 1}, {1, -1, -1}}};
inline const std::array<c::Vec3, 3> FlatT3{{
    {0, 0, 0}, {2, .25, .5}, {.25, 1.5, -.25}}};

}  // namespace current_regularity_test
