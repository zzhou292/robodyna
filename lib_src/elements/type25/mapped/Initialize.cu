// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Startup.h"
#include "../Type25BatchStorage.h"
#include <new>
#include <stdexcept>

namespace tl::fea::type25 {
BatchReport Batch::InitializeMapped(const BatchConfig& config,const ShellPhysicalBinding& physical,
    FENodalState& owner,const NodalCinWitnessSource& witnesses,CapacityProfile profile) try {
  if (impl_) return {BatchStatus::InvalidInput,"TYPE25 batch is already initialized"};
  mapped::Forecast forecast;
  auto report=mapped::MakeForecast(config,physical,witnesses,profile,sizeof(Impl),forecast);
  if (report.status!=BatchStatus::Success) return report;
  const auto& model=*physical.coefficients()->type25();
  report=mapped::ValidateModel(model);
  if (report.status!=BatchStatus::Success) return report;
  // Source-order endpoint checks occur once at initialization. Do not impose
  // this profile's rotational or non-rigid requirements on unrelated domain nodes.
  for (std::size_t element=0;element<model.connection_count();++element) {
    const auto* nodes=model.connections()[element].global_node;
    auto roles=owner.ValidateNonRigidNodes(nodes,2);
    if (roles.status==NodalStatus::Ok) roles=owner.ValidateFreeRotationalNodes(nodes,2);
    if (roles.status!=NodalStatus::Ok) return {
        roles.status==NodalStatus::DeviceFailure?BatchStatus::DeviceFailure:BatchStatus::InvalidInput,
        roles.message,static_cast<std::uint32_t>(element),roles.node,Status::Success,roles.status};
  }
  const auto proof=shell_physical_owner::AuthenticateInitial(*physical.coefficients(),owner,
      config.owner,config.startup,witnesses,forecast.proof);
  if (proof.status!=NodalStatus::Ok) return {
      proof.status==NodalStatus::DeviceFailure?BatchStatus::DeviceFailure:BatchStatus::InvalidInput,
      proof.message,UINT32_MAX,proof.node,Status::Success,proof.status};
  util::HostArena arena;
  if (!arena.Initialize(forecast.device.bytes)) {
    return {BatchStatus::ResourceLimit,"Mapped TYPE25 host arena allocation failed"};
  }
  batch_detail::Storage header;
  BatchDiagnostics diagnostics;
  report=mapped::BuildModel(config,physical,arena,forecast.device,header,diagnostics);
  if (report.status!=BatchStatus::Success) return report;
  auto next=std::make_unique<Impl>();
  next->config=config;
  next->accepted_stamp=config.owner;
  next->layout=forecast.device;
  next->source.emplace(model);
  next->physical.emplace(physical);
  next->cin_witness_count=witnesses.witness_count;
  next->accepted_diagnostics=diagnostics;
  next->staging=std::make_unique<Evaluation[]>(config.element_count);
  next->host_payload_bytes=forecast.host_bytes;
  report=next->Upload(arena,header);
  if (report.status!=BatchStatus::Success) return report;
  impl_=std::move(next);
  return {BatchStatus::Success,"Mapped TYPE25 physical owner prepared"};
} catch (const std::bad_alloc&) {
  return {BatchStatus::ResourceLimit,"Mapped TYPE25 host allocation failed"};
} catch (const std::length_error&) {
  return {BatchStatus::ResourceLimit,"Mapped TYPE25 host allocation extent overflow"};
}
} // namespace tl::fea::type25
