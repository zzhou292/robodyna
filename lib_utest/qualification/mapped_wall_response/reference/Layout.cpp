// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Layout.h"
namespace tlfea::contact::nodal_wall_mapped {
bool MakeLayout(std::size_t parents,std::size_t nodes,std::size_t groups,
    std::size_t cap,Layout& output) noexcept {
  if(!parents || !nodes || parents>MaxVehicleNodalWallDeviceParents ||
      nodes>MaxVehicleNodalWallDeviceNodes || groups>1024) return false;
  tl::util::BoundedArenaLayout builder(cap);
  Layout next;
  if(!builder.Append<std::uint8_t>(parents,next.accepted) ||
      !builder.Append<std::uint8_t>(parents,next.proposed) ||
      !builder.Append<std::uint32_t>(nodes,next.roots) ||
      !builder.Append<RigidContactBody>(groups,next.bodies) ||
      !builder.Append<double>(groups,next.traces) ||
      !builder.Append<double>(nodes,next.stiffness) ||
      !builder.Append<double>(nodes,next.inverse) ||
      !builder.Append<Summary>(1,next.summary) ||
      !builder.Append<ObserverSummary>(ObserverBlocks(nodes),next.observer) ||
      !builder.Append<IntervalSummary>(ObserverBlocks(nodes),next.interval)) return false;
  next.bytes=builder.bytes();
  output=next;
  return true;
}
Sidecar Bind(void* base,const Layout& layout) noexcept {
  using tl::util::ArenaPointer;
  return {ArenaPointer<std::uint8_t>(base,layout.accepted),
    ArenaPointer<std::uint8_t>(base,layout.proposed),
    ArenaPointer<std::uint32_t>(base,layout.roots),
    ArenaPointer<RigidContactBody>(base,layout.bodies),
    ArenaPointer<double>(base,layout.traces),ArenaPointer<double>(base,layout.stiffness),
    ArenaPointer<double>(base,layout.inverse),ArenaPointer<Summary>(base,layout.summary),
    layout.bodies.count,ArenaPointer<ObserverSummary>(base,layout.observer),
    ArenaPointer<IntervalSummary>(base,layout.interval)};
}
} // namespace tlfea::contact::nodal_wall_mapped
