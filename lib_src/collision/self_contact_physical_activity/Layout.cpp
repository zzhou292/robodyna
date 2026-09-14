// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"

namespace tlfea::contact::self_contact_physical_activity {

bool MakeLayout(std::size_t selected, std::size_t qeph,
                std::size_t t3, std::size_t qbat,
                std::size_t maximum_bytes, Layout& output) noexcept {
  if (!selected || !maximum_bytes) return false;
  tl::util::BoundedArenaLayout builder(maximum_bytes);
  Layout next;
  if (!builder.Append<std::uint8_t>(selected, next.accepted) ||
      !builder.Append<std::uint8_t>(selected, next.current) ||
      !builder.Append<std::uint8_t>(qeph, next.qeph) ||
      !builder.Append<std::uint8_t>(t3, next.t3) ||
      !builder.Append<std::uint8_t>(qbat, next.qbat))
    return false;
  next.bytes = builder.bytes();
  output = next;
  return true;
}

Buffers Bind(void* base, const Layout& layout) noexcept {
  return {
      tl::util::ArenaPointer<std::uint8_t>(base, layout.accepted),
      tl::util::ArenaPointer<std::uint8_t>(base, layout.current),
      tl::util::ArenaPointer<std::uint8_t>(base, layout.qeph),
      tl::util::ArenaPointer<std::uint8_t>(base, layout.t3),
      tl::util::ArenaPointer<std::uint8_t>(base, layout.qbat)};
}

}  // namespace tlfea::contact::self_contact_physical_activity
