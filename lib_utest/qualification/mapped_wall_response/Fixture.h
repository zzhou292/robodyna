// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "FrozenResponse.h"
#include "lib_src/collision/nodal_wall_mapped/ResponseValues.h"
#include "lib_src/collision/nodal_wall_mapped/ResponseIncidence.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cstring>
#include <limits>
#include <vector>

namespace wall_response_test {
namespace c = tlfea::contact;
namespace d = c::nodal_wall_device_detail;
namespace m = c::nodal_wall_mapped;
namespace r = m::response;
namespace fe = tl::fea;
struct Packet {
  unsigned count, groups;
  d::Storage storage;
  m::Summary summary;
  std::vector<c::NodalWallPointResult> nodes;
  std::vector<c::RigidContactBody> bodies;
  std::vector<double> position, inverse, traces, maxima;
  std::vector<std::uint32_t> roots, offsets, rows;
  explicit Packet(unsigned n = 263, unsigned g = 7) : count(n), groups(g), nodes(n), bodies(g),
      position(3*(2*n+1)), inverse(n), traces(g), maxima(r::Blocks(n)), roots(n), offsets(g+1), rows(n) {
    Reset();
  }
  void Reset() {
    storage = {};
    summary = {};
    storage.model.node_count = count;
    storage.model.config.owner.fixed_dt = 1e-8;
    storage.result.nodes = nodes.data();
    storage.result.diagnostics.stiffness_rate_bound = -719.125;
    storage.result.diagnostics.resultant.value = -0.;
    summary.rate = -37.25;
    summary.removed_potential.value = 31.5;
    summary.interval_tree_used = true;
    std::fill(traces.begin(), traces.end(), -81.5);
    std::fill(maxima.begin(), maxima.end(), NAN);
    for (unsigned g = 0; g < groups; ++g) {
      bodies[g] = {};
      bodies[g].mass = 2+.25*g;
      bodies[g].current_frame.inertia = {3+.125*g, 4+.25*g, 5+.5*g};
      bodies[g].current_frame.axes = {{1, 0, 0, 0, 1, 0, 0, 0, 1}};
      bodies[g].center = {.01*g, -.005*g, .003*g};
    }
    for (unsigned i = 0; i < count; ++i) {
      nodes[i] = {};
      nodes[i].node = 2*(count-1-i); // First error is compact order, not domain ID.
      nodes[i].stiffness.value = .125+(i%13);
      nodes[i].stiffness.upper = nodes[i].stiffness.value+.03125;
      roots[i] = groups && i%3 == 0 ? (i/3)%groups : UINT32_MAX;
      inverse[i] = .25+.125*(i%7);
      auto* x = position.data()+3*nodes[i].node;
      x[0] = .125;
      x[1] = .25+.015625*(i%11);
      x[2] = -.5-.0078125*(i%19);
    }
    EXPECT_TRUE(r::BuildIncidence(roots.data(), count, groups, Side().response));
  }
  fe::NodalAssemblyView View() {
    fe::NodalAssemblyView view;
    view.accepted.position_xyz = position.data();
    return view;
  }
  m::Sidecar Side() {
    m::Sidecar side;
    side.roots = roots.data();
    side.bodies = bodies.data();
    side.traces = traces.data();
    side.inverse = inverse.data();
    side.summary = &summary;
    side.groups = groups;
    side.response = {offsets.data(), rows.data(), maxima.data(), count, maxima.size()};
    return side;
  }
};
inline std::uint64_t Bits(double value) {
  std::uint64_t result;
  std::memcpy(&result, &value, sizeof(value));
  return result;
}
inline void Same(const Packet& a, const Packet& b) {
  EXPECT_EQ(a.storage.control.status, b.storage.control.status);
  EXPECT_EQ(a.storage.control.node, b.storage.control.node);
  EXPECT_EQ(a.storage.control.parent, b.storage.control.parent);
  EXPECT_EQ(a.storage.control.point.status, b.storage.control.point.status);
  EXPECT_EQ(a.storage.control.point.cause, b.storage.control.point.cause);
  EXPECT_EQ(Bits(a.summary.rate), Bits(b.summary.rate));
  EXPECT_EQ(a.summary.parent_failure, b.summary.parent_failure);
  EXPECT_EQ(Bits(a.summary.removed_potential.value), Bits(b.summary.removed_potential.value));
  EXPECT_EQ(a.summary.interval_tree_used, b.summary.interval_tree_used);
  EXPECT_EQ(Bits(a.storage.result.diagnostics.stiffness_rate_bound), Bits(b.storage.result.diagnostics.stiffness_rate_bound));
  EXPECT_EQ(Bits(a.storage.result.diagnostics.resultant.value), Bits(b.storage.result.diagnostics.resultant.value));
  ASSERT_EQ(a.traces.size(), b.traces.size());
  for (unsigned g = 0; g < a.groups; ++g) EXPECT_EQ(Bits(a.traces[g]), Bits(b.traces[g])) << g;
}
inline void Serial(Packet& p) { wall_response_frozen::CheckResponse(&p.storage, p.Side(), p.View()); }
inline void Staged(Packet& p) {
  if (p.storage.control.status != c::NodalWallDeviceStatus::Ok) return;
  p.summary.parent_failure = r::NoFailure;
  auto side = p.Side();
  for (unsigned b = 0; b < p.maxima.size(); ++b) {
    double maximum = 0;
    for (unsigned lane = 0; lane < r::Threads; ++lane)
      for (unsigned i = b*r::Threads+lane; i < p.count; i += p.maxima.size()*r::Threads)
        if (!r::Ordinary(p.storage, side, i, maximum)) p.summary.parent_failure = 0;
    p.maxima[b] = maximum;
  }
  for (unsigned g = 0; g < p.groups; ++g)
    if (!r::Rigid(p.storage, side, p.View().accepted, g)) p.summary.parent_failure = 0;
  if (r::CompleteRate(p.storage, side) || wall_response_frozen::Response(p.storage, side, p.View().accepted))
    r::CheckStep(p.storage, side);
}
inline void Fault(Packet& p, unsigned fault) {
  const auto last = p.count-1; // Default last row is ordinary.
  switch (fault) {
    case 0: p.inverse[last] = INFINITY; break;
    case 1: p.inverse[last] = std::numeric_limits<double>::max(); break;
    case 2: p.inverse[last] = std::numeric_limits<double>::denorm_min();
      p.nodes[last].stiffness.value = p.nodes[last].stiffness.upper = .125; break;
    case 3: p.roots[last] = p.groups; break;
    case 4: p.bodies[2].mass = 0; break;
    case 5: p.position[3*p.nodes[21].node+2] = NAN; break;
    case 6: p.bodies[1].current_frame.inertia.y = -1; break;
    case 7:
      for (auto row : {21u, 42u})
        p.nodes[row].stiffness.value = p.nodes[row].stiffness.upper = std::numeric_limits<double>::max();
      break;
    case 8: p.inverse[1] = INFINITY; p.bodies[2].mass = 0; p.roots[last] = p.groups; break;
    case 9: p.bodies[0].mass = 0; p.inverse[1] = INFINITY; break;
    case 10: p.storage.model.config.owner.fixed_dt = 1e6; break;
    case 11: p.storage.model.config.owner.fixed_dt = INFINITY; break;
    case 12: p.roots[1] = p.groups; p.bodies[2].mass = 0; break;
  }
}
} // namespace wall_response_test
