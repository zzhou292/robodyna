// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Arena.h"

namespace tl::fea::type45::resident_detail {
BatchReport Plan(const BatchConfig& c,const Model& model,ArenaLayout& output) noexcept {
  const auto& o=c.owner;
  if(!model.prepared() || !model.domain() || !model.rigid_binding() ||
      !o.owner_id || o.epoch || o.time!=0 || o.velocity_time!=0 || !o.has_rotations ||
      o.reactions_valid || o.reaction_base_epoch || o.reaction_time!=0 || o.reaction_kick_dt!=0 ||
      o.temporal_scheme!=NodalTemporalScheme::StaggeredHalfKickStart ||
      o.velocity_phase!=NodalVelocityPhase::Collocated || !detail::Positive(o.fixed_dt) ||
      !c.configuration_id || !c.qualification_id || !shell_startup_detail::ValidStartup(c.startup,true,true) ||
      c.profile!=BatchProfile::PhysicalAggregateV1 || !c.cin_witness_count ||
      c.cin_witness_count>NodalCinLimits{}.max_witnesses ||
      o.node_count!=model.domain()->node_count() ||
      o.rigid_groups.group_count!=model.rigid_binding()->groups().size() ||
      o.rigid_groups.member_count!=model.rigid_binding()->members().size())
    return {BatchStatus::InvalidInput,"Joint participant requires a fresh complete physical CIN profile"};
  if(model.owned_payload_bytes()>c.limits.max_host_bytes ||
      !MakeLayout(model.joints().size(),c,output))
    return {BatchStatus::ResourceLimit,"Joint model, counts or complete layout exceed limits"};
  return {};
}
} // namespace tl::fea::type45::resident_detail
