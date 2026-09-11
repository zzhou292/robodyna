// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "QbatBatchTypes.h"
#include "../ShellBatchArenaLayout.h"

namespace tl::fea::qbat::batch_detail {
struct Element {
  Reference reference;
  std::size_t nodes[4]{};
  std::uint64_t source_parent_id=0;
  Material material;
  Failure failure;
};
static_assert(sizeof(Element)==1072,"Immutable QBAT source/parameter row is budgeted exactly");
struct Model {
  BatchConfig config;
  Element* element=nullptr;
  Vec3* initial_position=nullptr;
  double *mass=nullptr,*inertia=nullptr,*physical=nullptr,*added=nullptr;
  double *curve_x=nullptr,*curve_y=nullptr;
  std::size_t curve_points=0;
  bool joined=true;
};
struct Slab { BatchResult* element=nullptr; };
struct Control {
  BatchStatus status=BatchStatus::Success;
  Status element_status=Status::kSuccess;
  std::uint32_t element=UINT32_MAX,node=UINT32_MAX;
  BatchDiagnostics diagnostics;
};
struct Storage {
  Model model;
  Slab slab[2];
  Control control;
  Status* candidate_status=nullptr;
};
static_assert(std::is_trivially_copyable_v<Storage>);
static_assert(sizeof(Storage)<=2048,"No capacity-sized device header fields");

// Existing typed model/result/node/slab layout, followed by one owned curve
// pool. This adds no three-point material or failure allocation.
struct Layout {
  using Common=shell_batch_detail::BatchArenaLayout<Storage,Element,BatchResult,Vec3,Status>;
  Common common;
  util::ArenaRegion curve_x,curve_y;
  std::size_t bytes=0;
  bool Initialize(std::size_t parents,std::size_t nodes,std::size_t points,
                  std::size_t cap) noexcept;
  Storage* Construct(util::HostArena&) const noexcept;
  Storage Rebase(const Storage&,void* device) const noexcept;
};
} // namespace tl::fea::qbat::batch_detail
