// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "Values.h"
#include "../normal_activation/Values.h"
#include "../selection/lifecycle/Admission.h"
#include <climits>
namespace tlfea::contact::radioss_type25::activity_operands::detail {
bool Disjoint(const void* a, std::size_t an, const void* b, std::size_t bn) noexcept {
  if (!an || !bn) return true;
  const auto x = reinterpret_cast<std::uintptr_t>(a), y = reinterpret_cast<std::uintptr_t>(b);
  return a && b && an <= UINTPTR_MAX-x && bn <= UINTPTR_MAX-y && (x+an <= y || y+bn <= x);
}
TransactionReport Check(const activity_source::Plan& plan, const ContactSourceInput& source,
    const current_normals::Topology* normal, UnitScale units, BorrowedSlot slot,
    bool pointers, Shape& output) noexcept {
  using S = TransactionStatus; namespace a = activity_source;
  if (!plan.initialized()) return {S::NotInitialized, "Contact activity source plan is not initialized"};
  const auto forecast = plan.forecast(); const auto v = plan.view(); const auto& c = forecast.counts;
  const auto& in = source.selection; units_detail::Factors factor;
  if (forecast.report.status != S::Ok || !units_detail::Make(units, factor) ||
      !c.nodes || !c.parents || !c.mains || !c.primaries || !in.secondary_count ||
      c.nodes >= UINT32_MAX || c.parents >= UINT32_MAX || c.mains >= INT_MAX ||
      in.secondary_count >= INT_MAX || in.node_count != c.nodes || in.main_count != c.mains ||
      source.primary_main_count != c.primaries || in.generation != v.source_generation ||
      v.parents.size() != c.parents || v.mains.size() != c.mains ||
      v.main_to_primary.size() != c.mains || v.node_offsets.size() != c.nodes+1 ||
      v.containing_offsets.size() != c.primaries+1 || v.emitting_offsets.size() != c.parents+1 ||
      v.emitting_mains.size() > c.emitting_capacity || v.emitting_mains.size() >= INT_MAX ||
      !lifecycle::detail::Span(in.mains, c.mains) || !lifecycle::detail::Span(in.secondary, in.secondary_count))
    return {S::SourceMismatch, "Contact operand source shape or generation differs"};
  if ((v.controls.deletion != a::Deletion::Disabled && v.controls.deletion != a::Deletion::ContainingElement) ||
      (v.controls.solid_erosion != startup::SolidErosion::Disabled && v.controls.solid_erosion != startup::SolidErosion::Enabled) ||
      (v.controls.deletion == a::Deletion::Disabled && v.controls.solid_erosion == startup::SolidErosion::Enabled))
    return {S::UnsupportedProfile, "Contact activity requires declared I_DEL0 or I_DEL1 and consistent erosion registration"};
  if (slot.main_capacity < c.mains || (normal && slot.free_capacity < c.mains) || (!normal && slot.free_capacity) ||
      slot.primary_capacity < c.primaries || slot.secondary_capacity < in.secondary_count ||
      (normal && slot.normal_capacity < c.mains) || (!normal && slot.normal_capacity))
    return {S::ResourceLimit, "Borrowed contact operand slot has incomplete capacity"};
  if (normal && (normal->main_count != c.mains || normal->primary_count != c.primaries ||
      normal->nodes != c.nodes || !lifecycle::detail::Span(normal->mains, c.mains)))
    return {S::SourceMismatch, "Current-normal topology differs from contact operands"};
  std::size_t free_count = 0;
  for (std::size_t i = 0; i < c.mains; ++i) {
    const auto& main = in.mains[i]; double scaled = 0;
    if (!tl::math::Finite(main.coefficient) || (i < c.primaries && !Scale(main.coefficient, factor.stiffness, scaled)) ||
        v.main_to_primary[i] >= c.primaries)
      return {S::InvalidInput, "Invalid native main coefficient or primary mapping", i};
    for (unsigned k = 0; k < 4; ++k) {
      if (main.nodes[k] >= c.nodes || main.neighbors[k] < 0 || std::size_t(main.neighbors[k]) > c.mains)
        return {S::InvalidInput, "Main connectivity or neighbor is out of range", i};
      if (normal && (normal->mains[i].nodes[k] != main.nodes[k] || normal->mains[i].neighbors[k] != main.neighbors[k]))
        return {S::SourceMismatch, "Normal and selection topology disagree", i};
    }
    if (normal) free_count += normal_activation::detail::FreeMain(main);
  }
  if (slot.initial_free_count != free_count)
    return {S::SourceMismatch, "Initial free roster count is not the complete native roster"};
  for (std::size_t i = 0; i < in.secondary_count; ++i) {
    double scaled = 0;
    if (in.secondary[i].node >= c.nodes || in.secondary[i].coefficient < 0 ||
        !Scale(in.secondary[i].coefficient, factor.stiffness, scaled))
      return {S::InvalidInput, "Invalid native secondary node or coefficient", i};
  }
  if (pointers) {
    struct Range { const void* pointer; std::size_t count, item, alignment; };
    const Range ranges[]{
      {slot.mains, c.mains, sizeof(lifecycle::Main), alignof(lifecycle::Main)},
      {slot.normal_mains, normal ? c.mains : 0, sizeof(startup::Main), alignof(startup::Main)},
      {slot.normal_coefficients, normal ? c.mains : 0, sizeof(double), alignof(double)},
      {slot.free_mains, normal ? c.mains : 0, sizeof(std::uint32_t), alignof(std::uint32_t)},
      {slot.main_stiffness_si, c.primaries, sizeof(double), alignof(double)},
      {slot.secondary_stiffness_si, in.secondary_count, sizeof(double), alignof(double)}};
    for (std::size_t i = 0; i < 6; ++i) {
      const auto& a = ranges[i]; if (!a.count) continue;
      const auto address = reinterpret_cast<std::uintptr_t>(a.pointer);
      if (!a.pointer || address%a.alignment || a.count > (UINTPTR_MAX-address)/a.item)
        return {S::InvalidInput, "Borrowed operand device range is null, misaligned or overflowing", i};
      for (std::size_t j = 0; j < i; ++j) if (!Disjoint(a.pointer, a.count*a.item,
          ranges[j].pointer, ranges[j].count*ranges[j].item))
        return {S::InvalidInput, "Borrowed operand writable ranges overlap", i};
    }
    if (!normal && (slot.normal_mains || slot.normal_coefficients || slot.free_mains))
      return {S::InvalidInput, "Disabled current normals supplied writable topology"};
  }
  output = {c.nodes, c.parents, c.mains, c.primaries, in.secondary_count,
      v.node_parents.size(), v.containing_parents.size(), v.emitting_mains.size(),
      c.families[std::size_t(a::Family::Qeph)], c.families[std::size_t(a::Family::T3)],
      c.families[std::size_t(a::Family::Qbat)], c.families[std::size_t(a::Family::Type45)], normal != nullptr};
  return {S::Ok, "Contact operand descriptors admitted"};
}
} // namespace tlfea::contact::radioss_type25::activity_operands::detail
namespace tlfea::contact::radioss_type25::activity_operands {
Forecast State::Preflight(const activity_source::Plan& plan, const ContactSourceInput& source,
    const current_normals::Topology* normal, UnitScale units, BorrowedSlot slot, Limits limits) noexcept {
  Forecast f; detail::Shape shape;
  f.report = detail::Check(plan, source, normal, units, slot, false, shape);
  if (f.report.status != TransactionStatus::Ok) return f;
  if (shape.normals && detail::ScanBytes(shape.mains, f.scan_workspace_bytes) != cudaSuccess) {
    f.report = {TransactionStatus::DeviceFailure, "Contact operand scan forecast failed"}; return f;
  }
  detail::Layout layout;
  if (!detail::MakeLayout(shape, f.scan_workspace_bytes, limits.max_device_bytes, layout)) {
    f.report = {TransactionStatus::ResourceLimit, "Contact activity device arena exceeds its cap"}; return f;
  }
  detail::StartupLayout staging;
  if (!detail::MakeStartupLayout(shape, limits.max_startup_host_bytes, staging)) {
    f.report = {TransactionStatus::ResourceLimit, "Contact activity upload staging exceeds its cap"}; return f;
  }
  tl::util::BoundedArenaLayout peak(limits.max_startup_host_bytes); tl::util::ArenaRegion ignored;
  f.owned_host_bytes = sizeof(State) + sizeof(Impl);
  f.source_plan_host_bytes = plan.forecast().output_bytes;
  // Upload staging and caller-owned source plan coexist. They are both gone
  // from this module after initialization; no retained host incidence copy.
  if (f.owned_host_bytes > limits.max_host_bytes ||
      !peak.Append<std::byte>(f.owned_host_bytes, ignored) ||
      !peak.Append<std::byte>(f.source_plan_host_bytes, ignored) ||
      !peak.Append<std::byte>(staging.bytes, ignored) ||
      !peak.Append<std::byte>(4096, ignored)) {
    f.report = {TransactionStatus::ResourceLimit, "Contact activity host startup exceeds its cap"}; return f;
  }
  f.startup_host_bytes = peak.bytes(); f.owned_device_bytes = layout.bytes;
  f.borrowed_device_bytes = shape.mains*sizeof(lifecycle::Main) +
      (shape.normals ? shape.mains*(sizeof(startup::Main)+sizeof(double)+sizeof(std::uint32_t)) : 0) +
      (shape.primaries+shape.secondaries)*sizeof(double);
  return f;
}
} // namespace tlfea::contact::radioss_type25::activity_operands
