// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Batch.h"
#include "../../../../lib_utils/BoundedArena.h"

namespace tl::fea::type13::batch_detail {
struct DeviceElement {
  Reference reference;
  Vec3 original_position_native[2]{};
  std::size_t nodes[2]{};
  std::size_t property = 0;
};
struct DeviceModel {
  BatchConfig config;
  WorkingUnits units;
  std::uint64_t source_instance_id = 0;
  std::size_t element_count = 0;
  Property* properties = nullptr;
  DeviceElement* elements = nullptr;
};
struct Control {
  BatchStatus status = BatchStatus::Success;
  Status element_status = Status::Success;
  std::size_t element = SIZE_MAX, node = SIZE_MAX;
  BatchDiagnostics diagnostics;
};
struct Storage {
  DeviceModel model;
  Evaluation* slab[2]{};
  Status* candidate_status = nullptr;
  Control control;
};
static_assert(std::is_trivially_copyable_v<Storage>);
static_assert(std::is_trivially_copyable_v<Property>);
static_assert(std::is_trivially_copyable_v<Evaluation>);
struct ArenaLayout {
  util::ArenaRegion header, properties, elements, slab[2], status;
  std::size_t bytes = 0;
};
bool MakeLayout(std::size_t properties, std::size_t elements,
                const BatchLimits&, ArenaLayout&) noexcept;
Storage RebasedHeader(void*, const ArenaLayout&) noexcept;
} // namespace tl::fea::type13::batch_detail
