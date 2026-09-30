// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/nodal_wall_mapped/AssemblyValidation.h"
#include "FrozenValidation.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cstring>
#include <limits>
#include <vector>
namespace wall_assembly_test {
namespace c = tlfea::contact;
namespace d = c::nodal_wall_device_detail;
namespace m = c::nodal_wall_mapped;
namespace a = m::assembly_validation;
namespace fe = tl::fea;
struct Packet {
  unsigned count, global;
  d::Storage storage;
  m::Summary summary;
  fe::NodalAssemblyResult result;
  fe::stability::RowBounds bounds;
  std::vector<c::NodalWallNodeWeight> nodes;
  std::vector<c::Vec3> initial;
  std::vector<double> position, inverse, copied;
  std::vector<std::uint8_t> fixed, translation;
  std::vector<std::uint32_t> roots;
  std::uint64_t epoch = 0;
  explicit Packet(unsigned n = 263) : count(n), global(2*n+1), nodes(n), initial(global),
      position(3*global), inverse(global), copied(n), fixed(global), translation(global), roots(n) {
    Reset();
  }
  void Reset() {
    storage = {};
    summary = {};
    result = {};
    bounds = {};
    result.base_epoch = bounds.base_epoch = epoch;
    result.attempt = bounds.attempt = 5;
    bounds.initialized = bounds.valid = true;
    storage.model.node_count = count;
    storage.model.nodes = nodes.data();
    storage.model.initial_position = initial.data();
    std::fill(copied.begin(), copied.end(), -91.25);
    std::fill(fixed.begin(), fixed.end(), 0);
    std::fill(translation.begin(), translation.end(), 0);
    for (unsigned n = 0; n < global; ++n) {
      initial[n] = {double(n), .125*n, -.25*n};
      position[3*n] = initial[n].x;
      position[3*n+1] = initial[n].y;
      position[3*n+2] = initial[n].z;
      inverse[n] = .25+.125*(n%7);
    }
    for (unsigned i = 0; i < count; ++i) {
      // Unique sparse rows in descending global order, deliberately unrelated
      // to the first-failure compact ordering.
      nodes[i].node = 2*(count-1-i);
      roots[i] = i%3 ? UINT32_MAX : 7;
      if (roots[i] != UINT32_MAX) inverse[nodes[i].node] = i%2 ? -0. : 0.;
    }
  }
  fe::NodalAssemblyView View() {
    fe::NodalAssemblyView view;
    view.result = &result;
    view.bounds = &bounds;
    view.attempt = 5;
    view.accepted.base_epoch = epoch;
    view.accepted.position_xyz = position.data();
    view.mass.inverse_mass = inverse.data();
    view.mass.fixed = fixed.data();
    view.translation_fixed_bits = translation.data();
    return view;
  }
  m::Sidecar Side() {
    m::Sidecar side;
    side.roots = roots.data();
    side.inverse = copied.data();
    side.summary = &summary;
    return side;
  }
};
inline std::uint64_t Bits(double value) {
  std::uint64_t result = 0;
  std::memcpy(&result, &value, sizeof(value));
  return result;
}
inline void Same(const Packet& left, const Packet& right) {
  EXPECT_EQ(left.storage.control.status, right.storage.control.status);
  EXPECT_EQ(left.storage.control.node, right.storage.control.node);
  EXPECT_EQ(left.storage.control.parent, right.storage.control.parent);
  EXPECT_EQ(left.storage.control.point.status, right.storage.control.point.status);
  EXPECT_EQ(left.storage.control.point.cause, right.storage.control.point.cause);
  EXPECT_EQ(left.summary.points_admitted, right.summary.points_admitted);
  EXPECT_EQ(left.summary.parent_failure, a::NoFailure);
  ASSERT_EQ(left.copied.size(), right.copied.size());
  for (unsigned i = 0; i < left.count; ++i) EXPECT_EQ(Bits(left.copied[i]), Bits(right.copied[i])) << i;
}
inline void Serial(Packet& p) {
  p.storage.control = {};
  p.summary = {};
  p.summary.points_admitted = wall_assembly_frozen::ValidateAssembly(p.storage, p.Side(), p.View());
}
inline void Staged(Packet& p) {
  p.storage.control = {};
  p.summary = {};
  const auto view = p.View();
  const auto side = p.Side();
  p.summary.points_admitted = a::Header(view, p.storage.control);
  if (p.summary.points_admitted) {
    for (unsigned index = 0; index < p.count; ++index) {
      const auto i = p.count-1-index; // Reverse execution must preserve first row.
      double inverse = 0;
      d::Control status;
      if (!a::Mass(p.storage, side, view, i, inverse, status))
        p.summary.parent_failure = std::min(p.summary.parent_failure, a::Failure(i, false));
      else if (!a::Geometry(p.storage, view, i, status))
        p.summary.parent_failure = std::min(p.summary.parent_failure, a::Failure(i, true));
    }
    const auto count = a::InversePrefix(p.count, p.summary.parent_failure);
    for (unsigned i = 0; i < count; ++i) p.copied[i] = p.inverse[p.nodes[i].node];
  }
  a::Complete(p.storage, side);
}
inline void Fault(Packet& p, unsigned fault) {
  const auto early = p.nodes[5].node;
  const auto late = p.nodes[p.count-1].node;
  if (fault == 0) p.fixed[late] = 1;
  if (fault == 1) p.translation[late] = 4;
  if (fault == 2) p.inverse[late] = NAN;
  if (fault == 3) p.inverse[late] = -1;
  if (fault == 4) { p.roots.back() = UINT32_MAX; p.inverse[late] = -0.; }
  if (fault == 5) p.position[3*late+2] = ::nextafter(p.position[3*late+2], INFINITY);
  if (fault == 6) { p.fixed[early] = 2; p.position[3*early] = NAN; p.inverse[late] = NAN; }
  if (fault == 7) { p.position[3*early+1] = NAN; p.fixed[late] = 1; }
  if (fault == 8) { p.result.attempt = 4; p.inverse[early] = NAN; }
  if (fault == 9) p.bounds.sealed = true;
  if (fault == 10) p.fixed[p.nodes[0].node] = 1;
  if (fault == 11) {
    p.position[3*p.nodes[0].node] = NAN;
    p.inverse[late] = -1;
  }
}
} // namespace wall_assembly_test
