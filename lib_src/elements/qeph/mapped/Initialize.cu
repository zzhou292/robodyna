// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Startup.h"
#include <new>
#include <stdexcept>

namespace tl::fea::qeph {
BatchReport QephBatch::InitializeMapped(const QephBatchConfig& config,const ShellPhysicalBinding& physical,
    FENodalState& owner,const NodalCinWitnessSource& source,const ShellBatchFailureLimits& limits) try {
  if (impl_) return {BatchStatus::InvalidInput,"Qeph batch is already initialized"};
  mapped::Forecast forecast;
  auto report=mapped::MakeForecast(config,physical,source,limits,sizeof(Impl),forecast);
  if (report.status!=BatchStatus::Success) return report;
  const auto map=physical.mapping()->mapping();
  auto roles=owner.ValidateFreeRotationalNodes(map.data(),map.size());
  if (roles.status==NodalStatus::Ok && physical.execution()) {
    roles=owner.ValidateRigidAssemblyBinding(*physical.execution()->rigid());
  }
  if (roles.status!=NodalStatus::Ok) return {
      roles.status==NodalStatus::DeviceFailure?BatchStatus::DeviceFailure:BatchStatus::InvalidInput,
      roles.message,UINT32_MAX,roles.node,Status::kSuccess,roles.status};
  const auto proof=shell_physical_owner::AuthenticateInitial(*physical.coefficients(),owner,
      config.owner,config.startup,source,forecast.proof);
  if (proof.status!=NodalStatus::Ok) return {
      proof.status==NodalStatus::DeviceFailure?BatchStatus::DeviceFailure:BatchStatus::InvalidInput,
      proof.message,UINT32_MAX,proof.node,Status::kSuccess,proof.status};
  util::HostArena arena;
  if (!arena.Initialize(forecast.device.bytes)) {
    return {BatchStatus::ResourceLimit,"Mapped Qeph host arena allocation failed"};
  }
  auto* host=forecast.device.Construct(arena);
  if (!host) return {BatchStatus::ResourceLimit,"Mapped Qeph host arena layout is invalid"};
  report=mapped::BuildModel(config,physical,*host);
  if (report.status!=BatchStatus::Success) return report;
  auto next=std::make_unique<Impl>();
  next->config=config;
  next->accepted_stamp=config.owner;
  next->layout=forecast.device;
  next->physical.emplace(physical);
  next->joined_binding.emplace(*physical.shells());
  next->formulations=true;
  next->cin_witness_count=source.witness_count;
  next->accepted_diagnostics=batch_detail::InitialDiagnostics(config,true);
  next->staging.Resize(config.element_count);
  next->activity_staging.Resize(mapped::ActivityBytes(config.element_count));
  next->failure_activity_staging.Resize(mapped::ActivityBytes(config.element_count));
  report=next->PendingError();
  if (report.status!=BatchStatus::Success) return report;
  auto material=std::make_unique<shell_batch_plasticity_detail::HostStorage>();
  const auto setup=material->InitializeMappedCollection(*next->physical,ShellBindingFamily::Qeph,
      config.element_count,config.max_device_bytes-forecast.device.bytes,
      config.storage_limits.max_host_bytes,limits);
  if (setup.status==shell_batch_plasticity_detail::SetupStatus::DeviceFailure) {
    return next->Runtime(setup.cuda_status,setup.message);
  }
  if (setup.status!=shell_batch_plasticity_detail::SetupStatus::Success) {
    return {setup.status==shell_batch_plasticity_detail::SetupStatus::ResourceLimit?
        BatchStatus::ResourceLimit:BatchStatus::InvalidInput,setup.message};
  }
  next->plasticity=std::move(material);
  report=next->Runtime(cudaMalloc(reinterpret_cast<void**>(&next->storage),forecast.device.bytes),
      "Mapped Qeph device allocation failed");
  if (report.status!=BatchStatus::Success) return report;
  next->device_header=forecast.device.Rebase(*host,next->storage);
  *host=next->device_header;
  report=next->Runtime(cudaMemcpy(next->storage,arena.data(),forecast.device.bytes,cudaMemcpyHostToDevice),
      "Mapped Qeph initialization copy failed");
  if (report.status!=BatchStatus::Success) return report;
  next->accepted=&next->storage->slab[0];
  next->trial=&next->storage->slab[1];
  impl_=std::move(next);
  return {BatchStatus::Success,"Mapped Qeph initial physical owner prepared"};
} catch (const std::bad_alloc&) {
  return {BatchStatus::ResourceLimit,"Mapped Qeph host allocation failed"};
} catch (const std::length_error&) {
  return {BatchStatus::ResourceLimit,"Mapped Qeph host extent overflow"};
}
} // namespace tl::fea::qeph
