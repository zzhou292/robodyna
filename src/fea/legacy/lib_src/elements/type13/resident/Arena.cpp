// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Arena.h"

namespace tl::fea::type13::batch_detail {
bool MakeLayout(std::size_t properties, std::size_t elements,
                const BatchLimits& limits, ArenaLayout& output, std::size_t mapped_nodes) noexcept {
  const BatchLimits hard;
  if (!limits.max_connections || limits.max_connections > hard.max_connections ||
      !limits.max_properties || limits.max_properties > hard.max_properties ||
      !limits.max_nodes || limits.max_nodes > hard.max_nodes ||
      !limits.max_device_bytes || limits.max_device_bytes > hard.max_device_bytes ||
      !limits.max_host_bytes || limits.max_host_bytes > hard.max_host_bytes ||
      !properties || properties > limits.max_properties ||
      !elements || elements > limits.max_connections || mapped_nodes > limits.max_nodes) {
    return false;
  }
  ArenaLayout next;
  util::BoundedArenaLayout layout(limits.max_device_bytes);
  if (!layout.Append<Storage>(1, next.header) ||
      !layout.Append<Property>(properties, next.properties) ||
      !layout.Append<DeviceElement>(elements, next.elements) ||
      !layout.Append<Evaluation>(elements, next.slab[0]) ||
      !layout.Append<Evaluation>(elements, next.slab[1]) ||
      !layout.Append<Status>(elements, next.status) ||
      !layout.Append<Measurement>(elements, next.measurement)) {
    return false;
  }
  if (mapped_nodes && !mapped_connector::AppendLayout(layout, elements, mapped_nodes, next.assembly)) return false;
  next.bytes = layout.bytes();
  output = next;
  return true;
}

Storage RebasedHeader(void* base, const ArenaLayout& layout) noexcept {
  Storage next;
  next.model.properties = util::ArenaPointer<Property>(base, layout.properties);
  next.model.elements = util::ArenaPointer<DeviceElement>(base, layout.elements);
  next.slab[0] = util::ArenaPointer<Evaluation>(base, layout.slab[0]);
  next.slab[1] = util::ArenaPointer<Evaluation>(base, layout.slab[1]);
  next.candidate_status = util::ArenaPointer<Status>(base, layout.status);
  next.measurement = util::ArenaPointer<Measurement>(base, layout.measurement);
  next.assembly = mapped_connector::Rebase(base, layout.assembly);
  return next;
}
} // namespace tl::fea::type13::batch_detail
