// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
namespace tlfea::contact::radioss_type25::activity_operands::detail {
bool MakeStartupLayout(Shape shape, std::size_t cap, StartupLayout& output) noexcept {
  tl::util::BoundedArenaLayout a(cap); StartupLayout next;
  if (!a.Append<std::uint32_t>(shape.secondaries, next.secondary_nodes) ||
      !a.Append<double>(shape.secondaries, next.secondary_coefficients) ||
      !a.Append<std::int32_t>(shape.mains, next.connected)) return false;
  next.bytes = a.bytes(); output = next; return true;
}
bool MakeLayout(Shape s, std::size_t scan_bytes, std::size_t cap, Layout& output) noexcept {
  tl::util::BoundedArenaLayout a(cap); Layout l;
#define ADD(type, count, name) if (!a.Append<type>(count, l.name)) return false
  ADD(activity_source::ParentIdentity, s.parents, parents);
  ADD(std::uint32_t, s.nodes + 1, node_offsets); ADD(std::uint32_t, s.incidence, node_parents);
  ADD(std::uint32_t, s.mains, main_to_primary); ADD(std::uint32_t, s.primaries + 1, containing_offsets);
  ADD(std::uint32_t, s.containing, containing_parents); ADD(std::uint32_t, s.parents + 1, emitting_offsets);
  ADD(std::uint32_t, s.emitting, emitting_mains); ADD(std::uint32_t, s.secondaries, secondary_nodes);
  for (unsigned i = 0; i < 2; ++i) {
    ADD(double, s.secondaries, secondary_coefficients[i]); ADD(std::int32_t, s.mains, connected[i]);
  }
  ADD(lifecycle::Main, s.mains, mains); ADD(startup::Main, s.normals ? s.mains : 0, normal_mains);
  ADD(double, s.normals ? s.mains : 0, normal_coefficients); ADD(std::uint32_t, s.normals ? s.mains : 0, free_mains);
  ADD(double, s.primaries, main_si); ADD(double, s.secondaries, secondary_si);
  ADD(std::uint8_t, s.parents, parent_active); ADD(std::uint8_t, s.parents, parent_removed);
  ADD(std::uint8_t, s.nodes, node_active); ADD(std::uint32_t, s.mains, events);
  ADD(std::uint8_t, s.mains, removed); ADD(std::uint32_t, s.normals ? s.mains + 1 : 0, flags);
  ADD(std::uint32_t, s.normals ? s.mains + 1 : 0, offsets); ADD(Control, 1, control);
  tl::util::ArenaRegion padding;
  if (!a.Append<std::byte>((256 - a.bytes()%256)%256, padding)) return false;
  ADD(std::byte, scan_bytes, scan);
#undef ADD
  l.bytes = a.bytes(); output = l; return true;
}
Device Bind(void* base, const Layout& l, Shape shape, activity_source::Controls controls,
    units_detail::Factors units, BorrowedSlot borrowed) noexcept {
  Device d; d.shape = shape; d.controls = controls; d.units = units;
#define BIND(type, name) d.name = tl::util::ArenaPointer<type>(base, l.name)
  BIND(activity_source::ParentIdentity, parents);
  BIND(std::uint32_t, node_offsets); BIND(std::uint32_t, node_parents);
  BIND(std::uint32_t, main_to_primary); BIND(std::uint32_t, containing_offsets); BIND(std::uint32_t, containing_parents);
  BIND(std::uint32_t, emitting_offsets); BIND(std::uint32_t, emitting_mains); BIND(std::uint32_t, secondary_nodes);
  BIND(std::uint8_t, parent_active); BIND(std::uint8_t, parent_removed); BIND(std::uint8_t, node_active);
  BIND(std::uint8_t, removed); BIND(std::uint32_t, events); BIND(std::uint32_t, flags); BIND(std::uint32_t, offsets);
  BIND(Control, control); BIND(std::byte, scan); d.scan_bytes = l.scan.bytes;
#undef BIND
  d.slots[0] = {borrowed.mains, borrowed.normal_mains, borrowed.normal_coefficients, borrowed.free_mains,
      nullptr, borrowed.main_stiffness_si, borrowed.secondary_stiffness_si, nullptr};
  auto& alt = d.slots[1];
#define ALT(type, name, region) alt.name = tl::util::ArenaPointer<type>(base, l.region)
  ALT(lifecycle::Main, mains, mains);
  ALT(double, main_stiffness_si, main_si); ALT(double, secondary_stiffness_si, secondary_si);
  if (shape.normals) { ALT(startup::Main, normal_mains, normal_mains); ALT(double, normal_coefficients, normal_coefficients); ALT(std::uint32_t, free_mains, free_mains); }
#undef ALT
  for (unsigned i = 0; i < 2; ++i) {
    d.slots[i].secondary_coefficients = tl::util::ArenaPointer<double>(base, l.secondary_coefficients[i]);
    d.slots[i].connected = tl::util::ArenaPointer<std::int32_t>(base, l.connected[i]);
  }
  return d;
}
} // namespace tlfea::contact::radioss_type25::activity_operands::detail
