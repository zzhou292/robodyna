// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Startup.h"
#include "Stiffness.h"
#include "Incidence.h"
#include "../QephHistory.h"

namespace tl::fea::qeph::mapped {
BatchReport MakeForecast(const QephBatchConfig& config,const ShellPhysicalBinding& physical,
    const NodalCinWitnessSource& source,const ShellBatchFailureLimits& limits,
    std::size_t private_bytes,Forecast& output) noexcept {
  const auto checked=shell_mapped_detail::Validate(config.owner,config.startup,physical,source);
  if (checked.status!=NodalStatus::Ok) return {BatchStatus::InvalidInput,checked.message};
  if (!config.configuration_id || !config.qualification_id ||
      config.usage!=BatchUsage::CoupledForces ||
      config.element_count!=physical.shells()->qeph_count()) {
    return {BatchStatus::InvalidInput,"Mapped Qeph requires the complete coupled family"};
  }
  Forecast next;
  if (!ValidShellResidentLimits(config.storage_limits,config.element_count,
      config.owner.node_count,config.max_device_bytes) ||
      !next.device.InitializeMapped(config.element_count,config.owner.node_count,config.max_device_bytes)) {
    return {BatchStatus::ResourceLimit,"Mapped Qeph active device layout exceeds its cap"};
  }
  const auto host_cap=config.storage_limits.max_host_bytes;
  std::size_t sidecar_bytes=0;
  if (!shell_physical_owner::ForecastProof(config.owner.node_count,source.range_count,host_cap,next.proof) ||
      !shell_mapped_detail::ForecastSections(physical,ShellBindingFamily::Qeph,config.element_count,
          config.max_device_bytes-next.device.bytes,host_cap,limits,sidecar_bytes,next.section_device_bytes)) {
    return {BatchStatus::ResourceLimit,"Mapped Qeph complete point/proof storage exceeds its cap"};
  }
  util::BoundedArenaLayout host(host_cap);
  util::ArenaRegion ignored;
  if (!host.Append<unsigned char>(private_bytes,ignored) ||
      !host.Append<unsigned char>(physical.owned_payload_bytes(),ignored) ||
      !host.Append<unsigned char>(next.device.bytes,ignored) ||
      !host.Append<ForceTrial>(config.element_count,ignored) ||
      !host.Append<std::uint8_t>(ActivityBytes(config.element_count),ignored) ||
      !host.Append<std::uint8_t>(ActivityBytes(config.element_count),ignored) ||
      !host.Append<std::uint8_t>(ActivityBytes(config.element_count),ignored) ||
      !host.Append<unsigned char>(sidecar_bytes,ignored) ||
      !host.Append<unsigned char>(next.proof.bytes,ignored) ||
      !host.Append<unsigned char>(4*64,ignored)) {
    return {BatchStatus::ResourceLimit,"Mapped Qeph complete startup/retained host payload exceeds its cap"};
  }
  next.host_bytes=host.bytes();
  output=next;
  return {BatchStatus::Success,"OK"};
}
BatchReport BuildModel(const QephBatchConfig& config,const ShellPhysicalBinding& physical,
    batch_detail::Storage& storage) noexcept {
  auto& model=storage.model;
  model.config=config;
  model.joined=true;
  model.mapped=true;
  if (!shell_mapped_detail::CopyLedger(physical,model)) {
    return {BatchStatus::NonfiniteResult,"Mapped Qeph complete kinetic domain overflow"};
  }
  for (std::size_t parent=0;parent<config.element_count;++parent) {
    auto& element=model.element[parent];
    // The immutable complete binding owns the qualified reference and exact
    // source-parent assignment. Distinct source layers can share node sets.
    element.reference=physical.shells()->qeph_reference(parent);
    const auto& nodes=physical.shells()->qeph_nodes(parent);
    for (unsigned slot=0;slot<4;++slot) {
      element.nodes[slot]=physical.mapping()->owner_index(nodes[slot]);
    }
    ShellSectionLaw law=ShellSectionLaw::Unspecified;
    if (!physical.catalog()->Law(ShellBindingFamily::Qeph,parent,&law)) {
      return {BatchStatus::InvalidInput,"Mapped Qeph parent law is unavailable",static_cast<std::uint32_t>(parent)};
    }
    const bool skin=law==ShellSectionLaw::RigidSkin;
    const auto* role=physical.execution()?physical.execution()->parent(ShellBindingFamily::Qeph,parent):nullptr;
    if (skin && (!role || role->law!=law || role->material_points!=0)) {
      return {BatchStatus::InvalidInput,"Mapped Qeph skin lacks explicit PART role",static_cast<std::uint32_t>(parent)};
    }
    const auto* failure=physical.failure()->parent(ShellBindingFamily::Qeph,parent);
    if (!failure || (skin && failure->policy!=ShellFailurePolicy::None) ||
        (!skin && element.reference.input.placement!=ShellReferencePlacement::Centered &&
         failure->policy!=ShellFailurePolicy::Tab1AnyPoint)) {
      return {BatchStatus::InvalidInput,"Mapped Qeph placement/failure role is unavailable",static_cast<std::uint32_t>(parent)};
    }
    ShellGlobalLaw1Profile profile;const ShellGlobalLaw1Profile* global=nullptr;
    if(law==ShellSectionLaw::GlobalLaw1Npt0) {
      if(!physical.catalog()->GlobalLaw1Profile(ShellBindingFamily::Qeph,parent,&profile))
        return {BatchStatus::InvalidInput,"Global LAW1 startup profile is unavailable",static_cast<std::uint32_t>(parent)};
      global=&profile;
    }
    NodalStiffness stiffness;
    if (!skin && !InitialStiffness(element.reference,law,stiffness,global)) {
      return {BatchStatus::ElementFailure,"Mapped Qeph virgin native stiffness is invalid",static_cast<std::uint32_t>(parent)};
    }
    const auto status=InitializeHistory(element.reference,{0,0},storage.slab[0].element[parent].proposed_history);
    if (status!=Status::kSuccess) {
      return {BatchStatus::ElementFailure,"Mapped Qeph endpoint bookkeeping is invalid",
          static_cast<std::uint32_t>(parent),UINT32_MAX,status};
    }
  }
  if (!BuildIncidence(model.element,config.element_count,config.owner.node_count,
      storage.assembly.offsets,config.owner.node_count+1,storage.assembly.incidence,4*config.element_count)) {
    return {BatchStatus::InvalidInput,"Mapped Qeph ordered incidence is invalid"};
  }
  return {BatchStatus::Success,"OK"};
}
} // namespace tl::fea::qeph::mapped

namespace tl::fea::qeph {
MappedForecast QephBatch::ForecastMapped(const QephBatchConfig& config,
    const ShellPhysicalBinding& physical,const NodalCinWitnessSource& source,const ShellBatchFailureLimits& limits) noexcept {
  mapped::Forecast next;
  MappedForecast result;
  result.report = mapped::MakeForecast(config,physical,source,limits,sizeof(Impl),next);
  if (result.report.status != BatchStatus::Success) return result;
  if (!shell_mapped_detail::MakeFootprint(next.host_bytes,physical.owned_payload_bytes(),
      next.device.bytes + next.section_device_bytes,next.proof.bytes,result.footprint)) {
    result.report = {BatchStatus::ResourceLimit,"Mapped footprint partition exceeds complete budget"};
  }
  return result;
}
} // namespace tl::fea::qeph
