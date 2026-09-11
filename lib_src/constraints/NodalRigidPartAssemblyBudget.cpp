// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalRigidPartAssemblyInternal.h"

namespace tl::fea::rigid::part_assembly_detail {
Report Preflight(const NodalRigidPartTopology& t,const NodalCoefficientLedger& c,
    NodalRigidSourceUnits units,PartAssemblyLimits limits,std::size_t header,Layout& out) noexcept {
  if(!t.prepared()||!c.prepared())
    return Fail(S::InvalidInput,"Prepared rigid topology and coefficient ledger are required");
  const PartAssemblyLimits hard;
  if(!limits.max_parts||limits.max_parts>hard.max_parts||
      !limits.max_members||limits.max_members>hard.max_members||
      !limits.max_host_bytes||limits.max_host_bytes>hard.max_host_bytes||
      t.part_count()>limits.max_parts||t.member_count()>limits.max_members)
    return Fail(S::ResourceLimit,"Rigid aggregate counts or bounds exceed scope");
  if(t.source_instance_id()!=c.domain()->source_instance_id())
    return Fail(S::IdentityMismatch,"Rigid topology and coefficient source instances differ");
  if(!std::isfinite(units.mass_to_kg)||units.mass_to_kg<=0||
      !std::isfinite(units.length_to_m)||units.length_to_m<=0)
    return Fail(S::InvalidInput,"Rigid aggregate source units must be explicit and positive");
  util::BoundedArenaLayout arena(limits.max_host_bytes),owned(limits.max_host_bytes),
      startup(limits.max_host_bytes);
  util::ArenaRegion ignored;
  Layout next;
  const auto coefficient_bytes=c.owned_payload_bytes();
  // Topology's byte API excludes its handle, unlike the coefficient ledger.
  if(coefficient_bytes<sizeof(NodalCoefficientLedger)||
      t.startup_payload_bytes()<t.owned_payload_bytes()||
      !arena.Append<PartAssemblyMember>(t.member_count(),next.members)||
      !arena.Append<PartAssemblyOriginal>(t.part_count(),next.originals)||
      !arena.Append<PartAssemblyRoot>(t.root_count(),next.roots)||
      !arena.Append<std::size_t>(t.member_count(),next.lookup)||
      !owned.Append<unsigned char>(header,ignored)||
      !owned.Append<unsigned char>(coefficient_bytes-sizeof(NodalCoefficientLedger),ignored)||
      !owned.Append<unsigned char>(t.owned_payload_bytes(),ignored)||
      !owned.Append<unsigned char>(arena.bytes(),ignored)||
      !startup.Append<unsigned char>(owned.bytes(),ignored)||
      !startup.Append<unsigned char>(t.startup_payload_bytes()-t.owned_payload_bytes(),ignored)||
      !startup.Append<PartTopologyPartInput>(t.part_count(),ignored)||
      !startup.Append<PartTopologyExtraInput>(t.extra_count(),ignored)||
      !startup.Append<AssemblyMassPoint>(t.member_count(),ignored)||
      !startup.Append<unsigned char>(util::SourceIdentityIndex<0>::Bytes(t.part_count()),ignored))
    return Fail(S::ResourceLimit,"Complete rigid aggregate backing and startup scratch exceed cap");
  next.arena=arena.bytes();
  next.retained=owned.bytes();
  next.startup=startup.bytes();
  out=next;
  return {};
}
} // namespace tl::fea::rigid::part_assembly_detail
