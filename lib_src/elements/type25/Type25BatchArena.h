// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Type25Batch.h"
#include "../../../lib_utils/BoundedArena.h"

namespace tl::fea::type25::batch_detail {
struct DeviceElement { Reference reference;std::size_t nodes[2]{},property_index=0; };
struct DeviceNode { Vec3 reference{};double mass=0,inertia=0; };
struct DeviceModel {
  BatchConfig config;
  SourceUnits units;
  std::uint64_t source_instance_id=0;
  Property* properties=nullptr;
  DeviceElement* elements=nullptr;
  DeviceNode* nodes=nullptr;
};
struct Slab { Evaluation* element=nullptr; };
struct Control {
  BatchStatus status=BatchStatus::Success;
  Status element_status=Status::Success;
  std::uint32_t element=UINT32_MAX,node=UINT32_MAX;
  BatchDiagnostics diagnostics;
};
struct Storage { DeviceModel model;Slab slab[2];Status* candidate_status=nullptr;Control control; };
static_assert(std::is_trivially_copyable_v<Storage>);
struct ArenaLayout {
  util::ArenaRegion header,properties,elements,nodes,slab[2],status;
  std::size_t bytes=0;
};
bool MakeLayout(std::size_t properties,std::size_t elements,std::size_t nodes,std::size_t cap,ArenaLayout&) noexcept;
// Rebase one header to either a fresh host startup arena or its device copy.
// The returned header is always a host value; no device pointer is dereferenced.
Storage RebasedHeader(void* arena,const ArenaLayout&) noexcept;
} // namespace tl::fea::type25::batch_detail
