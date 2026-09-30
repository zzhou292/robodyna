// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Arena.h"

namespace tl::fea::beam18::batch_detail {
bool MakeLayout(Counts count, const BatchConfig& config, ArenaLayout& output) noexcept {
  const BatchLimits hard;
  const auto& limit = config.limits;
  if (!limit.max_parents || limit.max_parents > hard.max_parents ||
      !limit.max_materials || limit.max_materials > hard.max_materials ||
      !limit.max_curve_points || limit.max_curve_points > hard.max_curve_points ||
      !limit.max_nodes || limit.max_nodes > hard.max_nodes ||
      !limit.max_device_bytes || limit.max_device_bytes > hard.max_device_bytes ||
      !limit.max_host_bytes || limit.max_host_bytes > hard.max_host_bytes ||
      !count.parents || count.parents > limit.max_parents || !count.materials ||
      count.materials > count.parents || count.materials > limit.max_materials ||
      !count.curve_points || count.curve_points > limit.max_curve_points ||
      config.owner.node_count > limit.max_nodes) return false;
  util::BoundedArenaLayout device(limit.max_device_bytes), host(limit.max_host_bytes);
  ArenaLayout next;
  if (!device.Append<Storage>(1, next.header) ||
      !device.Append<Parent>(count.parents, next.parents) ||
      !device.Append<Material>(count.materials, next.materials) ||
      !device.Append<double>(2 * count.curve_points, next.curves) ||
      !device.Append<State>(count.parents, next.slab[0]) ||
      !device.Append<State>(count.parents, next.slab[1]) ||
      !device.Append<int>(count.parents, next.status) ||
      !host.Append<State>(count.parents, next.staging) ||
      !shell_physical_owner::ForecastProof(config.owner.node_count,
          config.cin_attachment_count, limit.max_host_bytes, next.proof)) return false;
  next.bytes = device.bytes();
  next.staging_bytes = host.bytes();
  next.curve_points = count.curve_points;
  output = next;
  return true;
}
Storage RebasedHeader(void* base, const ArenaLayout& layout) noexcept {
  Storage next;
  next.parents = util::ArenaPointer<Parent>(base, layout.parents);
  next.materials = util::ArenaPointer<Material>(base, layout.materials);
  next.slab[0] = util::ArenaPointer<State>(base, layout.slab[0]);
  next.slab[1] = util::ArenaPointer<State>(base, layout.slab[1]);
  next.status = util::ArenaPointer<int>(base, layout.status);
  next.count = layout.parents.count;
  return next;
}
} // namespace tl::fea::beam18::batch_detail
