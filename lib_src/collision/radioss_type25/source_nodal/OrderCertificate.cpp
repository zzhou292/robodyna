// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../../RadiossType25NodalCorrection.h"
#include "../search/Ranges.h"
#include "HostRanges.h"
#include "lib_src/math/ScalarBits.h"
#include "lib_utils/BoundedArena.h"
#include <new>

namespace tlfea::contact::radioss_type25::source_nodal::correction {
namespace {
struct Layout {
  tl::util::ArenaRegion factors, first;
  Forecast forecast;
};
Report Prepare(const OrderInput& input, Limits limits, Layout& result) noexcept {
  const Limits hard;
  if (!limits.nodes || limits.nodes > hard.nodes || limits.solids > hard.solids ||
      !limits.scratch_bytes || limits.scratch_bytes > hard.scratch_bytes ||
      input.node_count > limits.nodes || input.solid_count > limits.solids)
    return {Status::ResourceLimit};
  if (!input.node_count || !search::detail::Span(input.solids, input.solid_count))
    return {Status::InvalidInput};
  Layout next;
  tl::util::BoundedArenaLayout arena(limits.scratch_bytes);
  if (!arena.Append<double>(input.node_count, next.factors) ||
      !arena.Append<std::uint32_t>(input.node_count, next.first))
    return {Status::ResourceLimit};
  next.forecast = {arena.bytes(), sizeof(OrderCertificate)};
  for (std::size_t i = 0; i < input.solid_count; ++i) {
    const auto& solid = input.solids[i];
    if (solid.control != 1) continue;
    FactorResult factor;
    const auto status = EvaluateFactor(solid, &factor);
    if (status != Status::Ok) return {status, i};
    for (const auto node : solid.nodes)
      if (node >= input.node_count) return {Status::InvalidInput, i, SIZE_MAX, node};
  }
  result = next;
  return {Status::Ok};
}
bool Separate(const OrderInput& input, const Layout& layout, void* scratch,
    std::size_t bytes, OrderCertificate* output) noexcept {
  if (!scratch || reinterpret_cast<std::uintptr_t>(scratch) % alignof(std::max_align_t) ||
      bytes < layout.forecast.scratch_bytes || bytes > UINTPTR_MAX-reinterpret_cast<std::uintptr_t>(scratch) ||
      !search::detail::Span(output, std::size_t{1})) return false;
  using source_nodal::detail::Range;
  using source_nodal::detail::Disjoint;
  const Range reads[]{{&input, sizeof(input)}, {input.solids, input.solid_count*sizeof(Solid)}};
  const Range writes[]{{scratch, bytes}, {output, sizeof(*output)}};
  for (const auto write : writes)
    for (const auto read : reads)
      if (!Disjoint(write, read)) return false;
  return Disjoint(writes[0], writes[1]);
}
OrderReport Failure(Report report) noexcept {
  const auto status = report.status == Status::ResourceLimit ? OrderStatus::ResourceLimit :
      report.status == Status::NonfiniteResult ? OrderStatus::NonfiniteResult : OrderStatus::InvalidInput;
  return {status, report.solid, SIZE_MAX, report.node};
}
}
Report PreflightOrderCertificate(const OrderInput& input, Limits limits, Forecast& output) noexcept {
  Layout layout;
  const auto report = Prepare(input, limits, layout);
  if (report.status == Status::Ok) output = layout.forecast;
  return report;
}
OrderReport CertifyOrder(const OrderInput& input, Limits limits, void* scratch,
    std::size_t bytes, OrderCertificate* output) noexcept {
  Layout layout;
  const auto admitted = Prepare(input, limits, layout);
  if (admitted.status != Status::Ok) return Failure(admitted);
  if (!Separate(input, layout, scratch, bytes, output)) return {OrderStatus::InvalidInput};
  auto* factors = ::new (static_cast<void*>(tl::util::ArenaPointer<double>(scratch, layout.factors))) double[input.node_count];
  auto* first = ::new (static_cast<void*>(tl::util::ArenaPointer<std::uint32_t>(scratch, layout.first))) std::uint32_t[input.node_count];
  for (std::size_t node = 0; node < input.node_count; ++node) first[node] = UINT32_MAX;
  OrderCertificate next{input.node_count, input.solid_count, 0, 0};
  for (std::size_t i = 0; i < input.solid_count; ++i) {
    const auto& solid = input.solids[i];
    if (solid.control != 1) continue;
    ++next.controlled_solids;
    FactorResult factor;
    const auto status = EvaluateFactor(solid, &factor);
    if (status != Status::Ok) return Failure({status, i});
    for (const auto node : solid.nodes) {
      if (first[node] == UINT32_MAX) {
        first[node] = static_cast<std::uint32_t>(i);
        factors[node] = factor.value;
        ++next.affected_nodes;
      } else if (!tl::math::SameScalarBits(factors[node], factor.value)) {
        return {OrderStatus::NeedsNativeStorageOrder, first[node], i, node};
      }
    }
  }
  *output = next;
  return {OrderStatus::Ok};
}
} // namespace tlfea::contact::radioss_type25::source_nodal::correction
