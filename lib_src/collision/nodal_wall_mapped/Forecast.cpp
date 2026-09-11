// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include "lib_utils/SourceIdentityIndex.h"
#include <limits>
namespace tlfea::contact::nodal_wall_mapped {
namespace {
bool Add(std::size_t value,std::size_t& sum) {
  if(value>std::numeric_limits<std::size_t>::max()-sum) return false;
  sum+=value;
  return true;
}
}
NodalWallDeviceReport Preflight(const NodalWallDeviceConfig& config,const NodalWallWeights& weights,
    const NodalWallMappedSource& source,NodalWallMappedLimits limits,std::size_t owner_bytes,d::ArenaLayout& arena,
    Layout& sidecar,fe::shell_physical_owner::ProofLayout& proof,fe::ShellMappedFootprint& output) noexcept {
  const bool vehicle=config.limits.profile==NodalWallDeviceProfile::Vehicle;
  const auto parent_cap=vehicle?MaxVehicleNodalWallDeviceParents:MaxActiveNodalWallDeviceParents;
  const auto node_cap=vehicle?MaxVehicleNodalWallDeviceNodes:MaxActiveNodalWallDeviceNodes;
  if((config.limits.profile!=NodalWallDeviceProfile::Legacy && !vehicle) ||
      !config.limits.parents || config.limits.parents>parent_cap || !config.limits.nodes ||
      config.limits.nodes>node_cap || !config.limits.global_nodes || config.limits.global_nodes>node_cap ||
      !config.max_host_bytes || config.max_host_bytes>(vehicle?MaxVehicleNodalWallHostBytes:MaxNodalWallHostBytes) ||
      !limits.max_startup_host_bytes || limits.max_startup_host_bytes>(vehicle?(8ull<<30):(64ull<<20)))
    return {Code::ResourceLimit,"Invalid explicit mapped wall profile or byte limits"};
  const auto& stamp=config.owner;
  const auto& law=config.law;
  if(!stamp.owner_id || !stamp.has_rotations || stamp.epoch || stamp.time!=0 || stamp.velocity_time!=0 ||
      stamp.reactions_valid || stamp.temporal_scheme!=fe::NodalTemporalScheme::StaggeredHalfKickStart ||
      stamp.velocity_phase!=fe::NodalVelocityPhase::Collocated || !IsFinite(stamp.fixed_dt) || stamp.fixed_dt<=0 ||
      !config.wall_binding_id || !IsFinite(config.exposed_clearance) || config.exposed_clearance<=0 ||
      !IsFinite(law.wall_x) || !IsFinite(law.stiffness_per_area) || law.stiffness_per_area<=0 ||
      !IsFinite(law.maximum_penetration) || law.maximum_penetration<=0 ||
      !IsFinite(law.parent_force_error) || law.parent_force_error<=0 ||
      !IsFinite(law.parent_energy_error) || law.parent_energy_error<=0 ||
      !fe::shell_startup_detail::ValidStartup(source.identity.startup,true))
    return {Code::InvalidInput,"Mapped wall requires finite explicit law and a fresh staggered owner"};
  if(!weights.prepared() || !source.physical || !source.physical->prepared() ||
      !source.rigid || !source.rigid->prepared() || !source.publication ||
      !source.cin.model || !source.cin.model->prepared() ||
      !source.physical->coefficients()->Matches(*source.rigid->coefficients()) ||
      !source.physical->domain()->Matches(*source.cin.model->domain()) ||
      source.cin.range_count!=source.cin.model->rows().count || !source.cin.ranges ||
      !source.cin.witnesses || !source.cin.witness_count || source.cin.witness_count>262144 ||
      source.cin.range_count>65536 || !source.identity.configuration_id || !source.identity.qualification_id ||
      config.configuration_id!=source.identity.configuration_id ||
      config.qualification_id!=source.identity.qualification_id)
    return {Code::InvalidInput,"Incomplete physical wall source, CIN or common publication identity"};
  const auto& binding=*source.physical->shells();
  const auto p=weights.parent_count(),n=weights.node_count(),global=weights.global_node_count();
  if(p!=binding.qeph_count()+binding.t3_count()+binding.qbat_count() ||
      n!=binding.node_count() || global!=source.physical->domain()->nodes().size() ||
      global!=config.owner.node_count || p>config.limits.parents || n>config.limits.nodes ||
      global>config.limits.global_nodes || source.rigid->groups().size()>1024 ||
      !d::BuildArenaLayout(p,n,global,config.max_device_bytes,arena,config.limits.profile) ||
      !MakeLayout(p,n,source.rigid->groups().size(),config.max_device_bytes-arena.bytes,sidecar) ||
      !fe::shell_physical_owner::ForecastProof(global,source.cin.range_count,config.max_host_bytes,proof))
    return {Code::ResourceLimit,"Complete mapped contact counts or arena/proof budget exceeded"};
  // Both original handles have inclusive payload APIs. Discount rigid only
  // when execution retains the exact same public group/member backing.
  std::size_t source_bytes=source.physical->owned_payload_bytes();
  const auto* execution=source.physical->execution();
  const bool shared=execution && source.rigid->groups().size() && source.rigid->members().size() &&
      execution->rigid()->groups().data()==source.rigid->groups().data() &&
      execution->rigid()->members().data()==source.rigid->members().data();
  if(!shared && !Add(source.rigid->owned_payload_bytes(),source_bytes))
    return {Code::ResourceLimit,"Mapped source byte arithmetic overflow"};
  std::size_t retained=owner_bytes+256;
  using ParentArray=tl::util::BoundedStartupArray<Parent,0>;
  using ByteArray=tl::util::BoundedStartupArray<std::uint8_t,0>;
  using SnapshotArray=tl::util::BoundedStartupArray<fe::NodalRigidGroupSnapshot,0>;
  if(!Add(arena.bytes,retained) || !Add(sidecar.bytes,retained) ||
      !Add(ParentArray::ExtraBytes(p),retained) || !Add(ByteArray::ExtraBytes(p),retained) ||
      !Add(SnapshotArray::ExtraBytes(source.rigid->groups().size()),retained))
    return {Code::ResourceLimit,"Mapped retained byte arithmetic overflow"};
  std::size_t scratch=proof.bytes;
  // Positions, temporary domain role/index arrays, one source index and the
  // existing incidence/wall scratch are conservatively reserved together.
  if(!Add(3*sizeof(double)*global,scratch) || !Add((sizeof(std::uint32_t)+sizeof(std::size_t))*global,scratch) ||
      !Add(tl::util::SourceIdentityIndex<0>::Bytes(p),scratch) ||
      !Add(d::HostPreparationBytes(arena)-arena.bytes,scratch) || !Add(256*1024,scratch) ||
      retained>config.max_host_bytes || scratch>config.max_host_bytes-retained)
    return {Code::ResourceLimit,"Mapped contact local host payload and scratch exceed declared cap"};
  std::size_t complete=source_bytes;
  if(!Add(retained,complete) || !Add(scratch,complete) || complete>limits.max_startup_host_bytes)
    return {Code::ResourceLimit,"Complete mapped contact startup reservation exceeds declared cap"};
  output={arena.bytes+sidecar.bytes,source_bytes,retained,scratch,complete};
  return {Code::Ok,"Complete mapped contact source and allocation forecast"};
}
} // namespace tlfea::contact::nodal_wall_mapped
namespace tlfea::contact {
NodalWallDeviceReport NodalWallMappedContact::Forecast(const NodalWallDeviceConfig& config,
    const NodalWallWeights& weights,const NodalWallMappedSource& source,
    tl::fea::ShellMappedFootprint& output,NodalWallMappedLimits limits) noexcept {
  nodal_wall_device_detail::ArenaLayout arena;
  nodal_wall_mapped::Layout sidecar;
  tl::fea::shell_physical_owner::ProofLayout proof;
  tl::fea::ShellMappedFootprint next;
  const auto report=nodal_wall_mapped::Preflight(config,weights,source,limits,sizeof(NodalWallMappedContact)+sizeof(Impl),arena,sidecar,proof,next);
  if(report.status==NodalWallDeviceStatus::Ok) output=next;
  return report;
}
} // namespace tlfea::contact
