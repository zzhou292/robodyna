// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../../constraints/tied_shell/runtime/CinStageTypes.h"
#include <cstdint>

namespace tl::fea::cin_advance::force_gather {
namespace cin = constraints::tied_shell::cin;
struct Master {
  double force[3]{};
  double mass = 0, inertia = 0, stiffness = 0;
};
enum Mode : std::uint32_t { Staging, NeedsSerial, Publish, SerialCompleted };
struct Summary {
  double numerical_mass = 0;
  cin::StageReport report;
  std::uint32_t mode = Mode::Staging;
};
static_assert(sizeof(Master) == 48 && sizeof(Summary) == 24);

// Created only from the admitted owner's bounded, disjoint arenas. A raw
// ForceTrial has no equivalent alias/topology authority and remains serial.
struct View {
  const cin::StageRow* source_rows = nullptr;
  const std::uint32_t* nodes = nullptr;
  const std::uint32_t* offsets = nullptr;
  const std::uint32_t* incidence = nullptr;
  Master* values = nullptr;
  Summary* summary = nullptr;
  std::uint32_t node_count = 0, row_count = 0, master_count = 0, capacity = 0;
};
TL_TIED_PATCH_HD inline bool Eligible(cin::StageView source, const View& view) noexcept {
  const auto capacity = source.node_count < 4ull * source.row_count
      ? source.node_count : 4ull * source.row_count;
  return source.rows && source.rows == view.source_rows && source.row_count &&
      source.row_count <= UINT32_MAX / 4 && source.node_count == view.node_count &&
      source.row_count == view.row_count && view.capacity == capacity &&
      view.master_count && view.master_count <= view.capacity && view.nodes &&
      view.offsets && view.incidence && view.values && view.summary;
}
} // namespace tl::fea::cin_advance::force_gather
