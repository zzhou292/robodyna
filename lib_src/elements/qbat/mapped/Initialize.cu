// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Startup.h"
#include "../QbatBatchStorage.h"
#include <new>
#include <stdexcept>

namespace tl::fea::qbat {
BatchReport Batch::InitializeMapped(const BatchConfig& config,const ShellPhysicalBinding& physical,
    FENodalState& owner,const NodalCinWitnessSource& source) try {
  if (impl_) return {BatchStatus::InvalidInput,"QBAT batch is already initialized"};
  mapped::Forecast forecast;
  auto report=mapped::MakeForecast(config,physical,source,sizeof(Impl),forecast);
  if (report.status!=BatchStatus::Success) return report;
  const auto map=physical.mapping()->mapping();
  const auto roles=(config.startup.kind==ShellBatchStartupKind::ReferenceConstrainedUniformTranslation ?
      owner.ValidatePresentRotationalNodes(map.data(),map.size()) :
      owner.ValidateFreeRotationalNodes(map.data(),map.size()));
  if (roles.status!=NodalStatus::Ok) return {
      roles.status==NodalStatus::DeviceFailure?BatchStatus::DeviceFailure:BatchStatus::InvalidInput,
      roles.message,UINT32_MAX,roles.node,Status::kSuccess,roles.status};
  // The actual complete owner is authenticated before any resident allocation.
  const auto proof=shell_physical_owner::AuthenticateInitial(*physical.coefficients(),owner,
      config.owner,config.startup,source,forecast.proof);
  if (proof.status!=NodalStatus::Ok) return {
      proof.status==NodalStatus::DeviceFailure?BatchStatus::DeviceFailure:BatchStatus::InvalidInput,
      proof.message,UINT32_MAX,proof.node,Status::kSuccess,proof.status};
  util::HostArena arena;
  if (!arena.Initialize(forecast.device.bytes)) {
    return {BatchStatus::ResourceLimit,"Mapped QBAT host arena allocation failed"};
  }
  auto* host=forecast.device.Construct(arena);
  if (!host) return {BatchStatus::ResourceLimit,"Mapped QBAT host arena layout is invalid"};
  BatchDiagnostics diagnostics;
  report=mapped::BuildModel(config,physical,*host,diagnostics);
  if (report.status!=BatchStatus::Success) return report;
  auto next=std::make_unique<Impl>();
  next->config=config;
  next->accepted_stamp=config.owner;
  next->layout=forecast.device;
  next->physical.emplace(physical);
  next->cin_witness_count=source.witness_count;
  next->accepted_diagnostics=diagnostics;
  next->staging=std::make_unique<BatchResult[]>(config.element_count);
  next->activity_staging.resize(mapped_shell::ActivityBytes(config.element_count));
  next->host_payload_bytes=forecast.host_bytes;
  report=next->Upload(arena,*host,*physical.catalog());
  if (report.status!=BatchStatus::Success) return report;
  impl_=std::move(next);
  return {BatchStatus::Success,"Mapped QBAT initial physical owner prepared"};
} catch (const std::bad_alloc&) {
  return {BatchStatus::ResourceLimit,"Mapped QBAT host allocation failed"};
} catch (const std::length_error&) {
  return {BatchStatus::ResourceLimit,"Mapped QBAT host allocation extent overflow"};
}
} // namespace tl::fea::qbat
