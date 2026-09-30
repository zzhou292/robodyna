// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Startup.h"
#include <new>

namespace tl::fea::type13 {
BatchReport Batch::InitializeMapped(const BatchConfig& config, const ShellPhysicalBinding& physical,
    const NodalRigidAssemblyBinding& rigid, FENodalState& owner,
    const NodalCinWitnessSource& cin, BatchMappedLimits limits) try {
  if (impl_) return {BatchStatus::InvalidInput, "TYPE13 batch is already initialized"};
  mapped::Forecast forecast;
  auto report = mapped::MakeForecast(config, physical, rigid, cin, limits,
                                    sizeof(Batch) + sizeof(Impl), forecast);
  if (!report) return report;
  auto nodal = owner.ValidateRigidAssemblyBinding(rigid);
  if (nodal.status != NodalStatus::Ok) {
    return {nodal.status == NodalStatus::DeviceFailure ? BatchStatus::DeviceFailure : BatchStatus::InvalidInput,
            nodal.message, SIZE_MAX, nodal.node,
            Status::Success, nodal.status};
  }
  const auto& source = *physical.coefficients()->type13();
  // The original source census proves these endpoints are non-rigid. Enforce
  // that profile on this actual owner; unrelated absent rotations are allowed.
  // Free rotational DOFs here means present/unfixed, never positive inverse J.
  for (std::size_t element = 0; element < source.model()->connection_count(); ++element) {
    const std::size_t nodes[2] = {source.records()[2 * element].value.global_node,
                                source.records()[2 * element + 1].value.global_node};
    nodal = owner.ValidateNonRigidNodes(nodes, 2);
    if (nodal.status == NodalStatus::Ok) nodal = owner.ValidateFreeRotationalNodes(nodes, 2);
    if (nodal.status != NodalStatus::Ok) {
      return {nodal.status == NodalStatus::DeviceFailure ? BatchStatus::DeviceFailure : BatchStatus::InvalidInput,
              nodal.message, element, nodal.node, Status::Success, nodal.status};
    }
  }
  nodal = shell_physical_owner::AuthenticateInitial(*physical.coefficients(), owner,
      config.owner, config.startup, cin, forecast.proof);
  if (nodal.status != NodalStatus::Ok) {
    return {nodal.status == NodalStatus::DeviceFailure ? BatchStatus::DeviceFailure : BatchStatus::InvalidInput,
            nodal.message, SIZE_MAX, nodal.node, Status::Success, nodal.status};
  }
  util::HostArena arena;
  if (!arena.Initialize(forecast.device.bytes)) {
    return {BatchStatus::ResourceLimit, "Mapped TYPE13 startup arena allocation failed"};
  }
  batch_detail::Storage header;
  BatchDiagnostics diagnostics;
  report = batch_detail::BuildSourceValues(config, source, arena, forecast.device, header, diagnostics);
  if (!report) return report;
  auto next = std::make_unique<Impl>(config, source);
  next->physical = std::make_unique<mapped::State>(physical, rigid, owner, cin.witness_count);
  next->layout = forecast.device;
  next->host_bytes = forecast.host_bytes;
  next->accepted_diagnostics = diagnostics;
  next->staging = std::make_unique<Evaluation[]>(source.model()->connection_count());
  report = next->Upload(arena, header);
  if (!report) return report;
  impl_ = std::move(next);
  return {};
} catch (const std::bad_alloc&) {
  return {BatchStatus::ResourceLimit, "Mapped TYPE13 host allocation failed"};
}
} // namespace tl::fea::type13
