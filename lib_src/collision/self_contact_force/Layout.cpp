// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "lib_src/solvers/NodalNativePhysicalCoefficients.h"

#include <algorithm>
#include <limits>

namespace tlfea::contact::self_contact_force {
namespace {

SelfContactForcePreflight Fail(SelfContactForceStatus status,
                               const char* message) noexcept {
  SelfContactForcePreflight result;
  result.report.status = status;
  result.report.message = message;
  return result;
}

bool Add(std::size_t value, std::size_t& total) noexcept {
  if (value > std::numeric_limits<std::size_t>::max() - total) return false;
  total += value;
  return true;
}

}  // namespace

bool MakeLayout(std::size_t events, std::size_t nodes,
                std::size_t max_bytes, Layout& output) noexcept {
  if (!events || !nodes || events > UINT32_MAX || nodes > UINT32_MAX ||
      events > std::numeric_limits<std::size_t>::max() / 8 ||
      nodes > std::numeric_limits<std::size_t>::max() / 6)
    return false;
  const std::size_t incidences = 8 * events;
  tl::util::BoundedArenaLayout builder(max_bytes);
  Layout next;
  if (!builder.Append<SelfContactForceEvent>(events, next.events) ||
      !builder.Append<SurfacePenaltyPacket>(events, next.packets) ||
      !builder.Append<SelfContactForceIncidence>(incidences, next.incidences) ||
      !builder.Append<SelfContactForceNodeIncidence>(nodes, next.nodes) ||
      !builder.Append<EventStatus>(events, next.event_status) ||
      !builder.Append<NodeStatus>(nodes, next.node_status) ||
      !builder.Append<double>(6 * nodes, next.staged_channels) ||
      !builder.Append<double>(nodes, next.staged_sti) ||
      !builder.Append<Control>(1, next.control) ||
      !builder.Append<SelfContactForceDiagnostics>(1, next.diagnostics))
    return false;
  next.bytes = builder.bytes();
  output = next;
  return true;
}

Buffers Bind(void* base, const Layout& layout) noexcept {
  using tl::util::ArenaPointer;
  return {
      ArenaPointer<SelfContactForceEvent>(base, layout.events),
      ArenaPointer<SurfacePenaltyPacket>(base, layout.packets),
      ArenaPointer<SelfContactForceIncidence>(base, layout.incidences),
      ArenaPointer<SelfContactForceNodeIncidence>(base, layout.nodes),
      ArenaPointer<EventStatus>(base, layout.event_status),
      ArenaPointer<NodeStatus>(base, layout.node_status),
      ArenaPointer<double>(base, layout.staged_channels),
      ArenaPointer<double>(base, layout.staged_sti),
      ArenaPointer<Control>(base, layout.control),
      ArenaPointer<SelfContactForceDiagnostics>(base, layout.diagnostics)};
}

SelfContactForcePreflight Preflight(
    const SelfContactForceConfig& config,
    const SelfContactActiveUseBinding& binding,
    SelfContactForceLimits limits,
    std::size_t owner_bytes,
    Layout* output_layout,
    tl::fea::shell_physical_owner::ProofLayout* output_proof) noexcept {
  using S = SelfContactForceStatus;
  if (!binding.prepared() || !binding.identity() || !binding.facets() ||
      !binding.facets()->surface() ||
      !binding.facets()->surface()->physical() ||
      !binding.facets()->surface()->physical()->prepared())
    return Fail(S::InvalidInput,
                "Complete active-use physical source is absent");
  const auto* physical = binding.facets()->surface()->physical();
  const auto cin = binding.cin();
  if (!cin.model || !cin.model->prepared() || !cin.ranges ||
      !cin.witnesses || !cin.range_count || !cin.witness_count ||
      cin.range_count != cin.model->rows().count)
    return Fail(S::InvalidInput,
                "Complete retained active-use CIN source is absent");
  const auto nodes = physical->domain()->node_count();
  if (!limits.max_events || !limits.max_nodes || !limits.max_host_bytes ||
      !limits.max_device_bytes || !limits.max_startup_host_bytes ||
      !config.event_capacity || config.event_capacity > limits.max_events ||
      config.event_capacity >
          std::numeric_limits<std::size_t>::max() / 8 ||
      !nodes || nodes > limits.max_nodes || nodes > UINT32_MAX ||
      config.owner.node_count != nodes || !config.owner.owner_id ||
      config.owner.epoch || config.owner.time != 0 ||
      config.owner.velocity_time != 0 || config.owner.reactions_valid ||
      !config.owner.has_rotations ||
      config.owner.temporal_scheme !=
          tl::fea::NodalTemporalScheme::StaggeredHalfKickStart ||
      config.owner.velocity_phase != tl::fea::NodalVelocityPhase::Collocated ||
      !IsFinite(config.owner.fixed_dt) || config.owner.fixed_dt <= 0 ||
      !tl::fea::shell_startup_detail::ValidStartup(config.startup, true) ||
      !IsFinite(config.stiffness_per_area_n_m3) ||
      config.stiffness_per_area_n_m3 <= 0 ||
      !config.configuration_id || !config.qualification_id)
    return Fail(S::InvalidInput,
                "Force config, fresh owner declaration or limits are invalid");
  if (binding.rigid()) {
    if (!binding.rigid()->prepared() ||
        !binding.rigid()->coefficients()->Matches(*physical->coefficients()))
      return Fail(S::IdentityMismatch,
                  "Retained rigid and physical sources differ");
  } else if (!tl::fea::native_physical_coefficients::Empty(
                 config.owner.rigid_groups)) {
    return Fail(S::IdentityMismatch,
                "Owner declares rigid groups absent from active-use authority");
  }

  const auto incidence_capacity = 8 * config.event_capacity;
  const auto touched_capacity = std::min(nodes, incidence_capacity);
  Layout layout;
  if (!MakeLayout(config.event_capacity, touched_capacity,
                  limits.max_device_bytes, layout) ||
      layout.bytes > limits.max_host_bytes)
    return Fail(S::ResourceLimit,
                "Force host/device arena exceeds declared capacity");
  tl::fea::shell_physical_owner::ProofLayout proof;
  if (!tl::fea::shell_physical_owner::ForecastProof(
          nodes, cin.range_count, limits.max_host_bytes, proof))
    return Fail(S::ResourceLimit,
                "Complete initial physical proof exceeds declared capacity");

  SelfContactForceForecast forecast;
  forecast.event_capacity = config.event_capacity;
  forecast.incidence_capacity = incidence_capacity;
  forecast.touched_node_capacity = touched_capacity;
  forecast.staged_channel_values = 6 * touched_capacity;
  forecast.host_arena_bytes = layout.bytes;
  forecast.device_bytes = layout.bytes;
  forecast.device_allocations = 1;
  const auto retained = binding.forecast().owned_payload_bytes;
  if (retained < sizeof(SelfContactActiveUseBinding))
    return Fail(S::IdentityMismatch,
                "Retained active-use byte forecast is invalid");
  forecast.retained_active_use_bytes =
      retained - sizeof(SelfContactActiveUseBinding);
  forecast.owned_host_bytes = owner_bytes;
  if (!Add(64, forecast.owned_host_bytes) ||
      !Add(layout.bytes, forecast.owned_host_bytes))
    return Fail(S::ResourceLimit,
                "Force retained host byte arithmetic overflow");
  forecast.startup_scratch_bytes = proof.bytes;
  forecast.startup_host_bytes = forecast.retained_active_use_bytes;
  if (!Add(forecast.owned_host_bytes, forecast.startup_host_bytes) ||
      !Add(forecast.startup_scratch_bytes, forecast.startup_host_bytes) ||
      forecast.startup_host_bytes > limits.max_startup_host_bytes)
    return Fail(S::ResourceLimit,
                "Complete force startup reservation exceeds declared cap");
  if (output_layout) *output_layout = layout;
  if (output_proof) *output_proof = proof;
  return {{}, forecast};
}

}  // namespace tlfea::contact::self_contact_force

namespace tlfea::contact {

SelfContactForcePreflight SelfContactForceAssembly::Forecast(
    const SelfContactForceConfig& config,
    const SelfContactActiveUseBinding& binding,
    SelfContactForceLimits limits) noexcept {
  return self_contact_force::Preflight(
      config, binding, limits,
      sizeof(SelfContactForceAssembly) + sizeof(Impl));
}

SelfContactForceForecast SelfContactForceAssembly::forecast() const noexcept {
  return impl_ ? impl_->storage_forecast : SelfContactForceForecast{};
}

}  // namespace tlfea::contact
