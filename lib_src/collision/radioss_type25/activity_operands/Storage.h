// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "State.h"
#include "lib_utils/BoundedArena.h"
namespace tlfea::contact::radioss_type25::activity_operands::detail {
struct Shape {
  std::size_t nodes = 0, parents = 0, mains = 0, primaries = 0, secondaries = 0;
  std::size_t incidence = 0, containing = 0, emitting = 0;
  std::size_t qeph = 0, t3 = 0, qbat = 0, type45 = 0;
  bool normals = false;
};
struct Slot {
  lifecycle::Main* mains = nullptr;
  startup::Main* normal_mains = nullptr;
  double* normal_coefficients = nullptr;
  std::uint32_t* free_mains = nullptr;
  double* secondary_coefficients = nullptr;
  double* main_stiffness_si = nullptr;
  double* secondary_stiffness_si = nullptr;
  std::int32_t* connected = nullptr;
};
struct Layout {
  tl::util::ArenaRegion parents, node_offsets, node_parents, main_to_primary;
  tl::util::ArenaRegion containing_offsets, containing_parents, emitting_offsets, emitting_mains;
  tl::util::ArenaRegion secondary_nodes, secondary_coefficients[2], connected[2];
  tl::util::ArenaRegion mains, normal_mains, normal_coefficients, free_mains, main_si, secondary_si;
  tl::util::ArenaRegion parent_active, parent_removed, node_active, events, removed;
  tl::util::ArenaRegion flags, offsets, control, scan;
  std::size_t bytes = 0;
};
struct StartupLayout {
  tl::util::ArenaRegion secondary_nodes, secondary_coefficients, connected;
  std::size_t bytes = 0;
};
bool MakeStartupLayout(Shape, std::size_t, StartupLayout&) noexcept;
struct Control {
  unsigned long long failure = UINT64_MAX, affected = 0, removed_events = 0;
  std::uint32_t changed = 0, removed_mains = 0, orphans = 0, free_count = 0;
};
enum class Failure : unsigned { Mask = 1, Source, Counter, Exposure, Scale };
struct Device {
  Shape shape;
  activity_source::Controls controls;
  units_detail::Factors units;
  const activity_source::ParentIdentity* parents = nullptr;
  const std::uint32_t *node_offsets = nullptr, *node_parents = nullptr;
  const std::uint32_t *main_to_primary = nullptr, *containing_offsets = nullptr, *containing_parents = nullptr;
  const std::uint32_t *emitting_offsets = nullptr, *emitting_mains = nullptr, *secondary_nodes = nullptr;
  Slot slots[2];
  std::uint8_t *parent_active = nullptr, *parent_removed = nullptr, *node_active = nullptr, *removed = nullptr;
  std::uint32_t *events = nullptr, *flags = nullptr, *offsets = nullptr;
  Control* control = nullptr;
  void* scan = nullptr;
  std::size_t scan_bytes = 0;
};
TransactionReport Check(const activity_source::Plan&, const ContactSourceInput&,
    const current_normals::Topology*, UnitScale, BorrowedSlot, bool pointers, Shape&) noexcept;
bool MakeLayout(Shape, std::size_t scan_bytes, std::size_t cap, Layout&) noexcept;
Device Bind(void*, const Layout&, Shape, activity_source::Controls, units_detail::Factors, BorrowedSlot) noexcept;
cudaError_t ScanBytes(std::size_t mains, std::size_t&) noexcept;
cudaError_t Launch(Device, unsigned accepted, const tl::fea::PhysicalActivityDeviceView&, cudaStream_t) noexcept;
TransactionReport Decode(const Control&) noexcept;
bool Disjoint(const void*, std::size_t, const void*, std::size_t) noexcept;
}
namespace tlfea::contact::radioss_type25::activity_operands {
struct State::Impl {
  ~Impl();
  detail::Shape shape;
  detail::Layout layout;
  detail::Device device;
  void* arena = nullptr;
  cudaStream_t stream = nullptr;
  Forecast forecast;
  detail::Control control;
  bool ready[2]{true, false}, usable = true;
  std::size_t free_count[2]{};
};
}
