// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Frozen.h"
#include "lib_src/collision/nodal_wall_mapped/RemovalEvents.h"
#include <gtest/gtest.h>
#include <cstring>
#include <limits>
#include <vector>

namespace wall_removal_test {
namespace c = tlfea::contact;
namespace d = c::nodal_wall_device_detail;
namespace m = c::nodal_wall_mapped;
namespace r = m::removal_events;
struct Packet {
  d::Storage storage;
  m::Summary summary;
  std::vector<std::uint8_t> accepted, proposed;
  std::vector<c::NodalWallParentResult> parents;
  explicit Packet(unsigned n = 777) : accepted(n), proposed(n), parents(n) { Reset(); }
  void Reset(unsigned pattern = 2) {
    // Define padding too, so the qualifier can compare the entire unchanged
    // control/summary/diagnostic packet, including the failure prefix.
    std::memset(&storage, 0, sizeof(storage));
    std::memset(&summary, 0, sizeof(summary));
    storage.control = {};
    storage.model.parent_count = static_cast<unsigned>(parents.size());
    storage.result.parents = parents.data();
    storage.result.diagnostics.owner_id = 713;
    storage.result.diagnostics.attempt = 7;
    storage.result.diagnostics.time = .125;
    storage.result.diagnostics.resultant = {17, 16, 18, 1};
    storage.result.diagnostics.kick_work = -0.;
    storage.result.diagnostics.stiffness_rate_bound = -19;
    summary.rate = 97;
    summary.parent_failure = 1234;
    summary.points_admitted = true;
    summary.interval_tree_used = true;
    summary.removed_potential = {11, 10, 12, 1};
    for (unsigned i = 0; i < parents.size(); ++i) {
      accepted[i] = pattern == 0 ? 0 : 1;
      proposed[i] = pattern == 1 || (pattern == 2 && i % 13 != 4) ? 1 : 0;
      parents[i] = {};
      const double value = .1 + .125 * (i % 7);
      parents[i].potential = {value, std::nextafter(value, 0.),
          std::nextafter(value, INFINITY), NAN}; // error is unconsumed by Sum.
      parents[i].parent_element_id = 3 * (parents.size() - i);
    }
  }
  m::Sidecar Side() {
    m::Sidecar side;
    side.accepted = accepted.data();
    side.proposed = proposed.data();
    side.summary = &summary;
    return side;
  }
};
inline void Same(const Packet& a, const Packet& b) {
  EXPECT_EQ(std::memcmp(&a.storage.control, &b.storage.control, sizeof(d::Control)), 0);
  EXPECT_EQ(std::memcmp(&a.storage.result.diagnostics, &b.storage.result.diagnostics,
      sizeof(c::NodalWallDiagnostics)), 0);
  EXPECT_EQ(std::memcmp(&a.summary, &b.summary, sizeof(m::Summary)), 0);
}
inline void Serial(Packet& p) {
  wall_removal_frozen::FinishCandidate(&p.storage, p.Side(), {});
}
inline void Staged(Packet& p) {
  if (p.storage.control.status != c::NodalWallDeviceStatus::Ok) return;
  auto side = p.Side();
  p.summary.removed_potential = {};
  for (unsigned first = 0; first < p.parents.size(); first += r::Threads) {
    r::Tile tile{};
    for (unsigned lane = 0; lane < r::Threads && first + lane < p.parents.size(); ++lane) {
      const auto event = r::Classify(p.accepted[first + lane], p.proposed[first + lane]);
      auto& word = tile.words[lane / r::WarpSize];
      if (event == r::Event::Invalid) word.invalid |= 1u << (lane % r::WarpSize);
      if (event == r::Event::Removed) word.removed |= 1u << (lane % r::WarpSize);
    }
    for (unsigned word = 0; word < r::Words; ++word)
      if (!r::FoldWord(p.storage, side, first + word * r::WarpSize, tile.words[word])) return;
  }
  p.storage.result.diagnostics.stiffness_rate_bound = p.summary.rate;
  p.storage.result.diagnostics.valid = true;
}
inline void Fault(Packet& p, unsigned fault) {
  p.Reset(3);
  const unsigned a = 254, b = 256;
  const double huge = std::numeric_limits<double>::max();
  switch (fault) {
    case 0: p.accepted[a] = 2; break;
    case 1: p.accepted[a] = 0; p.proposed[a] = 1; break;
    case 2: p.proposed[a] = 255; break;
    case 3: p.parents[a].potential.value = NAN; break;
    case 4: p.parents[a].potential.lower = -1; break;
    case 5: p.parents[a].potential.upper = INFINITY; break;
    case 6:
      p.parents[a - 1].potential = {huge, huge, huge, 0};
      p.parents[a].potential = {huge, huge, huge, 0};
      p.accepted[b] = 255;
      break;
    case 7: p.accepted[a] = 255; p.parents[b].potential.value = NAN; break;
    case 8: p.storage.control.status = c::NodalWallDeviceStatus::Accuracy;
      p.storage.control.parent = 19;
      p.storage.result.diagnostics.valid = true;
      p.accepted[a] = 255;
      p.storage.result.parents = nullptr;
      break;
    case 9:
      p.proposed[a] = 1;
      p.parents[a].potential = {NAN, NAN, NAN, NAN};
      break;
    case 10: p.parents[a].potential = {-0., -0., 0., NAN}; break;
    case 11: {
      const double tiny = std::numeric_limits<double>::denorm_min();
      p.parents[a].potential = {tiny, tiny, tiny, NAN};
      break;
    }
    case 12: p.parents[a].potential = {1, 2, 1, 0}; break;
  }
}
} // namespace wall_removal_test
