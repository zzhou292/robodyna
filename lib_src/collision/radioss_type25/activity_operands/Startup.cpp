// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include <new>
namespace tlfea::contact::radioss_type25::activity_operands {
State::State() noexcept = default;
State::~State() = default;
State::Impl::~Impl() { if (arena) cudaFree(arena); }
TransactionReport State::Initialize(const activity_source::Plan& plan, const ContactSourceInput& source,
    const current_normals::Topology* normal, UnitScale units, BorrowedSlot borrowed,
    cudaStream_t stream, Limits limits) noexcept try {
  using S = TransactionStatus;
  if (impl_) return {S::AlreadyInitialized, "Contact operands are already initialized"};
  if (!stream) return {S::InvalidInput, "Contact operands require the owner's explicit stream"};
  detail::Shape shape;
  auto report = detail::Check(plan, source, normal, units, borrowed, true, shape);
  if (report.status != S::Ok) return report;
  const auto forecast = Preflight(plan, source, normal, units, borrowed, limits);
  if (forecast.report.status != S::Ok) return forecast.report;
  auto next = std::make_unique<Impl>();
  next->shape = shape; next->stream = stream; next->forecast = forecast;
  if (!detail::MakeLayout(shape, forecast.scan_workspace_bytes, limits.max_device_bytes, next->layout))
    return {S::ResourceLimit, "Contact operand layout exceeds its cap"};
  detail::StartupLayout host_layout;
  tl::util::HostArena host;
  if (!detail::MakeStartupLayout(shape, limits.max_startup_host_bytes, host_layout) || !host.Initialize(host_layout.bytes))
    return {S::ResourceLimit, "Contact operand upload storage allocation failed"};
  auto* nodes = host.Construct<std::uint32_t>(host_layout.secondary_nodes);
  auto* coefficients = host.Construct<double>(host_layout.secondary_coefficients);
  auto* connected = host.Construct<std::int32_t>(host_layout.connected);
  if (!nodes || !coefficients || !connected) return {S::ResourceLimit, "Contact operand upload layout failed"};
  const auto plan_view = plan.view();
  for (std::size_t i = 0; i < shape.secondaries; ++i) {
    nodes[i] = source.selection.secondary[i].node; coefficients[i] = source.selection.secondary[i].coefficient;
  }
  for (std::size_t i = 0; i < shape.mains; ++i)
    connected[i] = source.selection.mains[i].coefficient < 0 ?
        1 + (plan_view.mains[i].second != activity_source::NoParent) : 0;
  if (cudaMalloc(&next->arena, next->layout.bytes) != cudaSuccess)
    return {S::ResourceLimit, "Contact operand device allocation failed"};
  units_detail::Factors factors; units_detail::Make(units, factors);
  next->device = detail::Bind(next->arena, next->layout, shape, plan_view.controls, factors, borrowed);
  const auto fail = [&]() {
    cudaStreamSynchronize(stream); // Drain borrowed/upload inputs before their lifetimes end.
    return TransactionReport{S::DeviceFailure, "Contact operand source upload failed"};
  };
  if (cudaMemsetAsync(next->arena, 0, next->layout.bytes, stream) != cudaSuccess) return fail();
  const auto upload = [&](const auto& input, const tl::util::ArenaRegion& region) {
    if (!region.bytes) return true;
    return cudaMemcpyAsync(static_cast<std::byte*>(next->arena)+region.offset,
        input.data(), region.bytes, cudaMemcpyHostToDevice, stream) == cudaSuccess;
  };
#define COPY(name) if (!upload(plan_view.name, next->layout.name)) return fail()
  COPY(parents); COPY(node_offsets); COPY(node_parents); COPY(main_to_primary);
  COPY(containing_offsets); COPY(containing_parents); COPY(emitting_offsets); COPY(emitting_mains);
#undef COPY
  auto& d = next->device;
  if (cudaMemcpyAsync(const_cast<std::uint32_t*>(d.secondary_nodes), nodes,
          shape.secondaries*sizeof(*nodes), cudaMemcpyHostToDevice, stream) != cudaSuccess ||
      cudaMemcpyAsync(d.slots[0].secondary_coefficients, coefficients,
          shape.secondaries*sizeof(*coefficients), cudaMemcpyHostToDevice, stream) != cudaSuccess ||
      cudaMemcpyAsync(d.slots[0].connected, connected,
          shape.mains*sizeof(*connected), cudaMemcpyHostToDevice, stream) != cudaSuccess ||
      cudaStreamSynchronize(stream) != cudaSuccess) return fail();
  next->free_count[0] = borrowed.initial_free_count;
  impl_ = std::move(next);
  return {S::Ok, "Contact operand source uploaded; caller retains slot 0"};
} catch (const std::bad_alloc&) {
  return {TransactionStatus::ResourceLimit, "Contact operand host allocation failed"};
}
Forecast State::allocations() const noexcept { return impl_ ? impl_->forecast : Forecast{}; }
bool State::OutputDisjoint(const void* output, std::size_t bytes) const noexcept {
  return impl_ && output && detail::Disjoint(output, bytes, this, sizeof(*this)) &&
      detail::Disjoint(output, bytes, impl_.get(), sizeof(*impl_)) &&
      detail::Disjoint(output, bytes, impl_->arena, impl_->layout.bytes);
}
View State::view(unsigned slot) const noexcept {
  View v;
  if (!impl_ || !impl_->usable || slot > 1 || !impl_->ready[slot]) return v;
  const auto& s = *impl_; const auto& d = s.device.slots[slot];
  v.mains = d.mains; v.normal_mains = d.normal_mains; v.normal_coefficients = d.normal_coefficients;
  v.free_mains = d.free_mains; v.secondary_coefficients = d.secondary_coefficients;
  v.connected_elements = d.connected; v.main_stiffness_si = d.main_stiffness_si;
  v.secondary_stiffness_si = d.secondary_stiffness_si;
  v.main_count = s.shape.mains; v.primary_count = s.shape.primaries;
  v.secondary_count = s.shape.secondaries; v.free_count = s.free_count[slot]; return v;
}
void State::DiscardStaged(unsigned accepted, unsigned alternate) noexcept {
  if (!impl_ || accepted > 1 || alternate > 1 || accepted == alternate) return;
  impl_->ready[alternate] = false;
}
} // namespace tlfea::contact::radioss_type25::activity_operands
