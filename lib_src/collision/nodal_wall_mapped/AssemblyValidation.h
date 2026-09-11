// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Layout.h"

namespace tlfea::contact::nodal_wall_mapped::assembly_validation {
namespace d = nodal_wall_device_detail;
namespace fe = tl::fea;
using Code = NodalWallDeviceStatus;
using Key = unsigned long long;
inline constexpr Key NoFailure = ~0ull;
inline constexpr unsigned Threads = 128;
inline constexpr unsigned MaximumBlocks = 256;

TL_SURFACE_HD inline bool Header(const fe::NodalAssemblyView& view, d::Control& status) {
  if(view.result->base_epoch!=view.accepted.base_epoch || view.result->attempt!=view.attempt ||
      view.bounds->base_epoch!=view.accepted.base_epoch || view.bounds->attempt!=view.attempt ||
      !view.bounds->initialized || !view.bounds->valid || view.bounds->sealed || view.result->status!=Status::kOk)
    return d::Fail(status,Code::AssemblyFailure);
  return true;
}
TL_SURFACE_HD inline bool Mass(const d::Storage& storage, Sidecar side,
    const fe::NodalAssemblyView& view, unsigned compact, double& inverse, d::Control& status) {
  const auto node=storage.model.nodes[compact].node;
  inverse=view.mass.inverse_mass[node];
  if(view.mass.fixed[node] || view.translation_fixed_bits[node] || !IsFinite(inverse) ||
      inverse<0 || (side.roots[compact]==UINT32_MAX && inverse<=0))
    return d::Fail(status,Code::InvalidMass,node);
  return true;
}
TL_SURFACE_HD inline bool Geometry(const d::Storage& storage,
    const fe::NodalAssemblyView& view, unsigned compact, d::Control& status) {
  if(view.accepted.base_epoch==0) {
    const auto node=storage.model.nodes[compact].node;
    const auto expected=storage.model.initial_position[node];
    const auto* x=view.accepted.position_xyz+3*node;
    if(x[0]!=expected.x || x[1]!=expected.y || x[2]!=expected.z)
      return d::Fail(status,Code::GeometryFailure,node);
  }
  return true;
}
TL_SURFACE_HD inline unsigned Blocks(std::size_t count) {
  if (!count || count > MaxVehicleNodalWallDeviceNodes) return 0;
  const auto blocks = 1+(count-1)/Threads;
  return static_cast<unsigned>(blocks > MaximumBlocks ? MaximumBlocks : blocks);
}
// Only compact row order matters; global domain IDs may be nonmonotonic.
// A geometry error follows the inverse write, whereas a mass error precedes it.
TL_SURFACE_HD inline Key Failure(unsigned compact, bool geometry) {
  return 2ull*compact+(geometry ? 1ull : 0ull);
}
TL_SURFACE_HD inline unsigned InversePrefix(unsigned count, Key failure) {
  return failure == NoFailure ? count : static_cast<unsigned>(failure/2+(failure&1));
}
TL_SURFACE_HD inline void Complete(d::Storage& storage, Sidecar side) {
  const auto failure = side.summary->parent_failure;
  if (side.summary->points_admitted && failure != NoFailure) {
    const auto compact = static_cast<unsigned>(failure/2);
    const auto code = failure&1 ? Code::GeometryFailure : Code::InvalidMass;
    d::Fail(storage.control, code, storage.model.nodes[compact].node);
    side.summary->points_admitted = false;
  }
  // This same integer word belongs to parent arbitration only after this
  // earlier phase has completed. No node failure may leak into parent indices.
  side.summary->parent_failure = NoFailure;
}
} // namespace tlfea::contact::nodal_wall_mapped::assembly_validation
