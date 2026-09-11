// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalCoefficientInternal.h"
#include "../../lib_utils/SourceIdentityIndex.h"

namespace tl::fea::coefficient_detail {
CoefficientReport Preflight(NodalCoefficientSources input,const ElementMassContributions* masses,CoefficientLimits limits,
    std::size_t implementation_bytes,Budget& result) noexcept {
  if(!input.shells||!input.shells->prepared()||
      (input.type25&&!input.type25->prepared())||
      (input.type13&&!input.type13->prepared())||(masses&&!masses->prepared()))
    return {S::InvalidInput,"Complete prepared typed sources are required"};
  const auto& shells=*input.shells->shells();
  const auto& domain=*input.shells->domain();
  const auto hard=CoefficientLimits::Vehicle();
  const auto parents=shells.qeph_count()+shells.t3_count()+shells.qbat_count();
  const auto springs=input.type25?input.type25->connection_count():0;
  const auto beams=input.type13?input.type13->model()->connection_count():0;
  const auto mass_records=masses?masses->records().size():0;
  if(!limits.max_nodes||limits.max_nodes>hard.max_nodes||
      !limits.max_shell_parents||limits.max_shell_parents>hard.max_shell_parents||
      !limits.max_type25_connections||limits.max_type25_connections>hard.max_type25_connections||
      !limits.max_type13_connections||limits.max_type13_connections>hard.max_type13_connections||
      !limits.max_element_mass_records||limits.max_element_mass_records>hard.max_element_mass_records||
      !limits.max_host_bytes||limits.max_host_bytes>hard.max_host_bytes||
      domain.node_count()>limits.max_nodes||parents>limits.max_shell_parents||
      springs>limits.max_type25_connections||beams>limits.max_type13_connections||
      mass_records>limits.max_element_mass_records)
    return {S::ResourceLimit,"Coefficient counts or caps exceed bounded scope"};

  util::BoundedArenaLayout retained(limits.max_host_bytes),peak(limits.max_host_bytes);
  util::ArenaRegion ignored;
  if(!retained.Append<unsigned char>(sizeof(NodalCoefficientLedger)+implementation_bytes+64,ignored)||
      !result.arena.Append<NodalCoefficientNode>(domain.node_count(),result.nodes)||
      !retained.Append<unsigned char>(result.arena.bytes(),ignored))
    return {S::ResourceLimit,"Coefficient node storage exceeds cap"};
  auto backing=[&](std::size_t bytes,std::size_t handle) {
    return bytes>=handle&&retained.Append<unsigned char>(bytes-handle,ignored);
  };
  if(!backing(input.shells->owned_payload_bytes(),sizeof(ShellNodeMap))||
      (input.type25&&!backing(input.type25->owned_payload_bytes(),sizeof(type25::Model))))
    return {S::ResourceLimit,"Retained coefficient producers exceed cap"};
  if(input.type13) {
    auto bytes=input.type13->owned_payload_bytes();
    if(domain.SharesStorage(*input.type13->domain())) {
      const auto shared=domain.owned_payload_bytes();
      if(shared<sizeof(NodalNodeDomain)||bytes<shared-sizeof(NodalNodeDomain))
        return {S::ResourceLimit,"Shared domain payload is inconsistent"};
      bytes-=shared-sizeof(NodalNodeDomain);
    }
    if(!backing(bytes,sizeof(Type13NodeContributions)))
      return {S::ResourceLimit,"Retained TYPE13 coefficient records exceed cap"};
  }
  if(masses) {
    auto bytes=masses->owned_payload_bytes();
    if(domain.SharesStorage(*masses->domain())) {
      const auto shared=domain.owned_payload_bytes();
      if(shared<sizeof(NodalNodeDomain)||bytes<shared-sizeof(NodalNodeDomain))
        return {S::ResourceLimit,"Shared element mass domain payload is inconsistent"};
      bytes-=shared-sizeof(NodalNodeDomain);
    } else if(input.type13&&input.type13->domain()->SharesStorage(*masses->domain())) {
      const auto shared=masses->domain()->owned_payload_bytes();
      if(shared<sizeof(NodalNodeDomain)||bytes<shared-sizeof(NodalNodeDomain))
        return {S::ResourceLimit,"Shared TYPE13/element mass domain payload is inconsistent"};
      bytes-=shared-sizeof(NodalNodeDomain);
    }
    if(!backing(bytes,sizeof(ElementMassContributions)))
      return {S::ResourceLimit,"Retained element mass records exceed cap"};
  }
  // One transient source index, released before publication; sorting never
  // changes source or reduction order. Retained plus scratch is the peak.
  if(!peak.Append<unsigned char>(retained.bytes(),ignored)||
      !peak.Append<unsigned char>(util::SourceIdentityIndex<0>::Bytes(parents+beams+mass_records),ignored))
    return {S::ResourceLimit,"Coefficient identity scratch exceeds startup cap"};
  result.retained=retained.bytes();
  result.startup=peak.bytes();
  return {};
}
} // namespace tl::fea::coefficient_detail
