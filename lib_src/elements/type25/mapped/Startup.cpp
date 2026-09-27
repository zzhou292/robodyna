// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Startup.h"
#include "../Type25BatchStorage.h"
#include "../Type25Property.h"

namespace tl::fea::type25::mapped {
BatchReport MakeForecast(const BatchConfig& config,const ShellPhysicalBinding& physical,
    const NodalCinWitnessSource& source,CapacityProfile profile,std::size_t private_bytes,
    Forecast& output) noexcept {
  const auto hard=Bounds(profile);
  if (!ValidProfile(profile) || !config.max_nodes || config.max_nodes>hard.nodes ||
      !config.max_connections || config.max_connections>hard.connections ||
      !config.max_host_bytes || config.max_host_bytes>hard.batch_host_bytes ||
      !config.max_device_bytes || config.max_device_bytes>hard.batch_device_bytes ||
      !config.owner.node_count || !config.element_count ||
      config.owner.node_count>config.max_nodes || config.element_count>config.max_connections) {
    return {BatchStatus::ResourceLimit,"Mapped TYPE25 explicit count/host limits are invalid"};
  }
  if (!physical.prepared() || !physical.coefficients()->type25() ||
      !source.model || !source.model->prepared()) {
    return {BatchStatus::InvalidInput,"Mapped TYPE25 requires physical TYPE25 and CIN sources"};
  }
  const auto& model=*physical.coefficients()->type25();
  const auto& owner=config.owner;
  if (!owner.owner_id || owner.epoch || owner.time!=0 || owner.velocity_time!=0 ||
      !owner.has_rotations || owner.temporal_scheme!=NodalTemporalScheme::StaggeredHalfKickStart ||
      owner.velocity_phase!=NodalVelocityPhase::Collocated || !detail::Positive(owner.fixed_dt) ||
      owner.reactions_valid || owner.reaction_base_epoch || owner.reaction_time!=0 ||
      owner.reaction_kick_dt!=0 || !config.configuration_id || !config.qualification_id ||
      !shell_startup_detail::ValidStartup(config.startup,true,true) ||
      config.element_count!=model.connection_count() || owner.node_count!=model.global_node_count() ||
      owner.node_count!=physical.domain()->node_count()) {
    return {BatchStatus::InvalidInput,"Mapped TYPE25 requires the complete model and fresh physical owner"};
  }
  if (!source.range_count || source.range_count!=source.model->rows().count ||
      source.range_count>NodalCinLimits{}.max_attachments || !source.witness_count ||
      source.witness_count>NodalCinLimits{}.max_witnesses || !source.ranges || !source.witnesses ||
      !source.model->domain()->SharesStorage(*physical.domain())) {
    return {BatchStatus::InvalidInput,"Mapped TYPE25 CIN roster/domain differs"};
  }
  Forecast next;
  if (!batch_detail::MakeLayout(model.property_count(),config.element_count,owner.node_count,
      config.max_device_bytes,next.device,profile,true) ||
      !shell_physical_owner::ForecastProof(owner.node_count,source.range_count,
          config.max_host_bytes,next.proof)) {
    return {BatchStatus::ResourceLimit,"Mapped TYPE25 arena/initial proof exceeds its cap"};
  }
  util::BoundedArenaLayout host(config.max_host_bytes);
  util::ArenaRegion ignored;
  // The physical ledger retains this exact model backing once. The embedded
  // source handle adds no independent model allocation. Proof and startup arena
  // are conservatively reserved together, even though their lifetimes differ.
  if (!host.Append<unsigned char>(private_bytes,ignored) ||
      !host.Append<unsigned char>(physical.owned_payload_bytes(),ignored) ||
      !host.Append<unsigned char>(next.device.bytes,ignored) ||
      !host.Append<Evaluation>(config.element_count,ignored) ||
      !host.Append<unsigned char>(next.proof.bytes,ignored) ||
      !host.Append<unsigned char>(64,ignored)) {
    return {BatchStatus::ResourceLimit,"Mapped TYPE25 complete startup payload exceeds its host cap"};
  }
  next.host_bytes=host.bytes();
  output=next;
  return {BatchStatus::Success,"OK"};
}

BatchReport ValidateModel(const Model& model) noexcept {
  if (!model.prepared()) return {BatchStatus::InvalidInput,"Mapped TYPE25 model is not prepared"};
  // Visit connections in source order so a late unsupported property reports
  // its first affected original connection rather than property catalog order.
  for (std::size_t element=0;element<model.connection_count();++element) {
    const auto& property=model.properties()[model.connections()[element].property_index].property;
    for (double damping:property.damping) {
      if (damping!=0) return {BatchStatus::InvalidInput,
          "Mapped TYPE25 currently requires zero damping for native nodal stiffness",
          static_cast<std::uint32_t>(element)};
    }
  }
  return {BatchStatus::Success,"OK"};
}

BatchReport BuildModel(const BatchConfig& config,const ShellPhysicalBinding& physical,
    util::HostArena& arena,const batch_detail::ArenaLayout& layout,
    batch_detail::Storage& output,BatchDiagnostics& diagnostics) {
  const auto& model=*physical.coefficients()->type25();
  const auto valid=ValidateModel(model);
  if (valid.status!=BatchStatus::Success) return valid;
  batch_detail::Storage next;
  auto report=batch_detail::ConstructStartup(config,model,arena,layout,next);
  if (report.status!=BatchStatus::Success) return report;
  const auto nodes=physical.domain()->nodes();
  const auto coefficients=physical.coefficients()->nodes();
  for (std::size_t node=0;node<config.owner.node_count;++node) {
    const auto& value=coefficients[node].coefficients;
    next.model.nodes[node]={nodes[node].position,value.mass,value.isotropic_inertia};
  }
  report=batch_detail::BuildElements(config,model,next,diagnostics);
  if (report.status!=BatchStatus::Success) return report;
  if (!mapped_connector::Build(next.model.elements, config.element_count,
          config.owner.node_count, layout.assembly, next.assembly))
    return {BatchStatus::InvalidInput,"Mapped TYPE25 incidence differs from source endpoints"};
  *util::ArenaPointer<batch_detail::Storage>(arena.data(),layout.header)=next;
  output=next;
  return {BatchStatus::Success,"OK"};
}
} // namespace tl::fea::type25::mapped

namespace tl::fea::type25 {
MappedForecast Batch::ForecastMapped(const BatchConfig& config,
    const ShellPhysicalBinding& physical,const NodalCinWitnessSource& source,
    CapacityProfile profile) noexcept {
  mapped::Forecast next;
  MappedForecast result;
  result.report = mapped::MakeForecast(config,physical,source,profile,sizeof(Impl),next);
  if (result.report.status != BatchStatus::Success) return result;
  if (!shell_mapped_detail::MakeFootprint(next.host_bytes,physical.owned_payload_bytes(),
      next.device.bytes,next.proof.bytes,result.footprint)) {
    result.report = {BatchStatus::ResourceLimit,"Mapped footprint partition exceeds complete budget"};
  }
  return result;
}
} // namespace tl::fea::type25
