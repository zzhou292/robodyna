// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Startup.h"
#include "../QbatBatchStorage.h"
#include <cmath>
#include "../../mapped_shell/Incidence.h"

namespace tl::fea::qbat::mapped {
BatchReport Validate(const BatchConfig& config,const ShellPhysicalBinding& physical,
    const NodalCinWitnessSource& source) noexcept {
  if (!physical.prepared() || !source.model || !source.model->prepared()) {
    return {BatchStatus::InvalidInput,"Mapped QBAT requires prepared physical and CIN sources"};
  }
  if (!ValidShellResidentLimits(config.storage_limits,config.element_count,
      config.owner.node_count,config.max_device_bytes)) {
    return {BatchStatus::ResourceLimit,"Mapped QBAT explicit resident limits are invalid"};
  }
  const auto& owner=config.owner;
  if (!owner.owner_id || !owner.has_rotations || owner.epoch || owner.time!=0 ||
      owner.velocity_time!=0 || owner.reactions_valid || !std::isfinite(owner.fixed_dt) ||
      owner.fixed_dt<=0 || owner.temporal_scheme!=NodalTemporalScheme::StaggeredHalfKickStart ||
      owner.velocity_phase!=NodalVelocityPhase::Collocated || !config.configuration_id ||
      !config.qualification_id || config.usage!=BatchUsage::CoupledForces ||
      config.element_count!=physical.shells()->qbat_count() ||
      owner.node_count!=physical.domain()->node_count() ||
      !shell_startup_detail::ValidStartup(config.startup,true,true)) {
    return {BatchStatus::InvalidInput,"Mapped QBAT requires the complete family and fresh coupled owner"};
  }
  if (source.range_count!=source.model->rows().count ||
      source.range_count>NodalCinLimits{}.max_attachments ||
      source.witness_count>NodalCinLimits{}.max_witnesses ||
      (source.model->explicitly_empty() ?
        (source.range_count || source.witness_count || source.ranges || source.witnesses) :
        (!source.range_count || !source.witness_count || !source.ranges || !source.witnesses)) ||
      !source.model->domain()->SharesStorage(*physical.domain())) {
    return {BatchStatus::InvalidInput,"Mapped QBAT CIN roster/domain differs or exceeds the supported scope"};
  }
  return {BatchStatus::Success,"OK"};
}

BatchReport MakeForecast(const BatchConfig& config,const ShellPhysicalBinding& physical,
    const NodalCinWitnessSource& source,std::size_t private_bytes,Forecast& output) noexcept {
  const auto valid=Validate(config,physical,source);
  if (valid.status!=BatchStatus::Success) return valid;
  Forecast next;
  if (!next.device.InitializeMapped(config.element_count,config.owner.node_count,
      physical.catalog()->curve_point_count(),config.max_device_bytes)) {
    return {BatchStatus::ResourceLimit,"Mapped QBAT device arena exceeds its explicit cap"};
  }
  if (!shell_physical_owner::ForecastProof(config.owner.node_count,source.range_count,
      config.storage_limits.max_host_bytes,next.proof)) {
    return {BatchStatus::ResourceLimit,"Mapped QBAT initial proof exceeds its host cap"};
  }
  util::BoundedArenaLayout host(config.storage_limits.max_host_bytes);
  util::ArenaRegion ignored;
  if (!host.Append<unsigned char>(private_bytes,ignored) ||
      !host.Append<unsigned char>(physical.owned_payload_bytes(),ignored) ||
      !host.Append<unsigned char>(next.device.bytes,ignored) ||
      !host.Append<BatchResult>(config.element_count,ignored) ||
      !host.Append<std::uint8_t>(util::BoundedStartupArray<std::uint8_t,0>::ExtraBytes(
          mapped_shell::ActivityBytes(config.element_count)),ignored) ||
      !host.Append<unsigned char>(next.proof.bytes,ignored) ||
      !host.Append<unsigned char>(64,ignored)) {
    return {BatchStatus::ResourceLimit,"Mapped QBAT complete retained/startup payload exceeds its host cap"};
  }
  next.host_bytes=host.bytes();
  output=next;
  return {BatchStatus::Success,"OK"};
}

BatchReport BuildModel(const BatchConfig& config,const ShellPhysicalBinding& physical,
    batch_detail::Storage& storage,BatchDiagnostics& output) noexcept {
  auto& model=storage.model;
  model.config=config;
  model.mapped=true;
  const auto source=physical.domain()->nodes();
  const auto coefficients=physical.coefficients()->nodes();
  double initial_kinetic=0;
  for (std::size_t node=0;node<config.owner.node_count;++node) {
    const auto& c=coefficients[node].coefficients;
    model.initial_position[node]=source[node].position;
    model.mass[node]=c.mass;
    model.inertia[node]=c.isotropic_inertia;
    model.physical[node]=c.shell.physical_inertia;
    model.added[node]=c.shell.added_inertia;
    if (config.startup.kind!=ShellBatchStartupKind::ReferenceConstrainedUniformTranslation &&
        !shell_startup_detail::AddInitialTranslationKinetic(c.mass,
        config.startup.uniform_velocity,initial_kinetic)) {
      return {BatchStatus::InvalidMass,"Mapped QBAT initial kinetic domain overflow",UINT32_MAX,
          static_cast<std::uint32_t>(node)};
    }
  }
  const ShellFormulationScope scope{physical.shells(),physical.catalog(),physical.failure(),nullptr};
  BatchDiagnostics next;
  const auto built=batch_detail::BuildElements(config,scope,storage,next);
  if (built.status!=BatchStatus::Success) return built;
  for (std::size_t parent=0;parent<config.element_count;++parent) {
    auto& element=model.element[parent];
    for (auto& node:element.nodes) node=physical.mapping()->owner_index(node);
  }
  if(!mapped_shell::BuildIncidence<4>(model.element,config.element_count,config.owner.node_count,
      storage.assembly.offsets,config.owner.node_count+1,storage.assembly.incidence,4*config.element_count)) {
    return {BatchStatus::InvalidInput,"Mapped QBAT source incidence is invalid"};
  }
  next.valid=true;
  output=next;
  return {BatchStatus::Success,"OK"};
}
} // namespace tl::fea::qbat::mapped

namespace tl::fea::qbat {
MappedForecast Batch::ForecastMapped(const BatchConfig& config,
    const ShellPhysicalBinding& physical,const NodalCinWitnessSource& source) noexcept {
  mapped::Forecast next;
  MappedForecast result;
  result.report = mapped::MakeForecast(config,physical,source,sizeof(Impl),next);
  if (result.report.status != BatchStatus::Success) return result;
  if (!shell_mapped_detail::MakeFootprint(next.host_bytes,physical.owned_payload_bytes(),
      next.device.bytes,next.proof.bytes,result.footprint)) {
    result.report = {BatchStatus::ResourceLimit,"Mapped footprint partition exceeds complete budget"};
  }
  return result;
}
} // namespace tl::fea::qbat
