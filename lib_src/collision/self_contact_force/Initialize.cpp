// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "lib_src/solvers/NodalTrialIdentity.h"

#include <new>

namespace tlfea::contact {
namespace fe = tl::fea;
namespace scf = self_contact_force;

SelfContactForceAssembly::SelfContactForceAssembly() noexcept = default;
SelfContactForceAssembly::~SelfContactForceAssembly() = default;

SelfContactForceAssembly::Impl::~Impl() {
  if (device) cudaFree(device);
}

SelfContactForceReport SelfContactForceAssembly::Impl::Check(
    cudaError_t code) noexcept {
  if (code == cudaSuccess) return {};
  usable = false;
  return {SelfContactForceStatus::DeviceFailure, SIZE_MAX, UINT64_MAX,
          UINT32_MAX, SurfacePenaltyStatus::InvalidInput,
          fe::NodalStatus::DeviceFailure, cudaGetErrorString(code)};
}

bool SelfContactForceAssembly::Impl::OutputDisjoint(
    const void* output, std::size_t bytes) const noexcept {
  using fe::trial_identity::Disjoint;
  return output && binding.OutputDisjoint(output, bytes) &&
      Disjoint(output, bytes, this, sizeof(*this)) &&
      Disjoint(output, bytes, host.data(), host.bytes());
}

SelfContactForceReport SelfContactForceAssembly::Initialize(
    const SelfContactForceConfig& config,
    const SelfContactActiveUseBinding& binding,
    fe::FENodalState& owner,
    SelfContactForceLimits limits) try {
  using S = SelfContactForceStatus;
  if (impl_)
    return {S::AlreadyInitialized, SIZE_MAX, UINT64_MAX, UINT32_MAX,
            SurfacePenaltyStatus::InvalidInput, fe::NodalStatus::Ok,
            "Self-contact force assembly is immutable"};
  if (!binding.OutputDisjoint(this, sizeof(*this)) ||
      !fe::trial_identity::Disjoint(
          this, sizeof(*this), &config, sizeof(config)) ||
      !fe::trial_identity::SameStamp(owner.accepted(), config.owner))
    return {S::IdentityMismatch, SIZE_MAX, UINT64_MAX, UINT32_MAX,
            SurfacePenaltyStatus::InvalidInput, fe::NodalStatus::Ok,
            "Force destination or fresh owner identity differs"};

  scf::Layout layout;
  fe::shell_physical_owner::ProofLayout proof;
  const auto preflight = scf::Preflight(
      config, binding, limits,
      sizeof(SelfContactForceAssembly) + sizeof(Impl), &layout, &proof);
  if (preflight.report.status != S::Ok) return preflight.report;

  if (binding.rigid()) {
    const auto rigid = owner.ValidateRigidAssemblyBinding(*binding.rigid());
    if (rigid.status != fe::NodalStatus::Ok)
      return {rigid.status == fe::NodalStatus::DeviceFailure
                  ? S::DeviceFailure : S::OwnerFailure,
              SIZE_MAX, UINT64_MAX, rigid.node,
              SurfacePenaltyStatus::InvalidInput, rigid.status,
              rigid.message};
  }
  const auto source = binding.cin();
  const fe::NodalCinWitnessSource cin{
      source.model, source.ranges, source.witnesses,
      source.range_count, source.witness_count};
  const auto* physical = binding.facets()->surface()->physical();
  const auto authenticated = fe::shell_physical_owner::AuthenticateInitial(
      *physical->coefficients(), owner, config.owner, config.startup,
      cin, proof);
  if (authenticated.status != fe::NodalStatus::Ok)
    return {authenticated.status == fe::NodalStatus::DeviceFailure
                ? S::DeviceFailure : S::OwnerFailure,
            SIZE_MAX, UINT64_MAX, authenticated.node,
            SurfacePenaltyStatus::InvalidInput, authenticated.status,
            authenticated.message};

  auto next = std::make_unique<Impl>(binding);
  next->owner = &owner;
  next->config = config;
  next->storage_forecast = preflight.forecast;
  next->layout = layout;
  if (!next->host.Initialize(layout.bytes))
    return {S::ResourceLimit, SIZE_MAX, UINT64_MAX, UINT32_MAX,
            SurfacePenaltyStatus::InvalidInput, fe::NodalStatus::Ok,
            "Force host arena allocation failed"};
  if (!next->host.Construct<SelfContactForceEvent>(layout.events) ||
      !next->host.Construct<SurfacePenaltyPacket>(layout.packets) ||
      !next->host.Construct<SelfContactForceIncidence>(
          layout.incidences) ||
      !next->host.Construct<SelfContactForceNodeIncidence>(
          layout.nodes) ||
      !next->host.Construct<scf::EventStatus>(layout.event_status) ||
      !next->host.Construct<scf::NodeStatus>(layout.node_status) ||
      !next->host.Construct<double>(layout.staged_channels) ||
      !next->host.Construct<double>(layout.staged_sti) ||
      !next->host.Construct<scf::Control>(layout.control) ||
      !next->host.Construct<SelfContactForceDiagnostics>(
          layout.diagnostics))
    return {S::ResourceLimit, SIZE_MAX, UINT64_MAX, UINT32_MAX,
            SurfacePenaltyStatus::InvalidInput, fe::NodalStatus::Ok,
            "Force typed host arena construction failed"};
  next->local = scf::Bind(next->host.data(), layout);
  auto checked = next->Check(
      cudaMalloc(&next->device, layout.bytes));
  if (checked.status != S::Ok) return checked;
  next->remote = scf::Bind(next->device, layout);
  checked = next->Check(cudaMemcpy(
      next->device, next->host.data(), layout.bytes,
      cudaMemcpyHostToDevice));
  if (checked.status != S::Ok) return checked;
  impl_ = std::move(next);
  return {};
} catch (const std::bad_alloc&) {
  return {SelfContactForceStatus::ResourceLimit, SIZE_MAX, UINT64_MAX,
          UINT32_MAX, SurfacePenaltyStatus::InvalidInput,
          fe::NodalStatus::Ok,
          "Self-contact force startup allocation failed"};
}

void SelfContactForceAssembly::DiscardTrial() noexcept {
  if (impl_) impl_->stream = nullptr;
}

fe::NodalAllocationInfo SelfContactForceAssembly::allocations() const noexcept {
  return impl_ ? fe::NodalAllocationInfo{
      impl_->storage_forecast.device_bytes,
      impl_->storage_forecast.device_allocations}
      : fe::NodalAllocationInfo{};
}

}  // namespace tlfea::contact
