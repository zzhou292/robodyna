// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Arena.h"
#include "../../../solvers/NodalNativePhysicalCoefficients.h"

namespace tl::fea::solids::batch_detail {
namespace {
bool DeclaredRigidScope(NodalRigidGroupInfo group, std::size_t nodes) noexcept {
  if (native_physical_coefficients::Empty(group)) return true;
  // Metadata bounds only. The actual complete PART/plain source association is
  // checked by ValidateRigidAssemblyBinding before this participant is claimed.
  return group.source_instance_id && group.group_count && group.member_count &&
      group.group_count <= 1024 && group.group_count <= group.member_count &&
      group.member_count <= nodes && group.part_group_count <= group.group_count &&
      (!group.part_group_count || (group.group_count == group.part_group_count
          ? group.plain_source_instance_id == 0 : group.plain_source_instance_id != 0));
}
}
BatchReport Plan(const BatchConfig& config, const Model& model,
    ArenaLayout& output) noexcept {
  const auto& owner = config.owner;
  const bool controlled_profile=config.profile==BatchProfile::PhysicalCinSourceControlsV3;
  if(model.control_selection()&&model.control_selection()->controlled_count()&&!controlled_profile)
    return {BatchStatus::InvalidInput, "Source-declared structural solid controls require a qualified controlled resident"};
  const bool original = config.profile == BatchProfile::PhysicalCinV1 &&
      model.profile() == ModelProfile::OriginalThreeFamilies;
  const bool extended = config.profile == BatchProfile::PhysicalCinExtendedLaw44Law90V2 &&
      model.profile() == ModelProfile::ExtendedLaw44Law90;
  if (!model.prepared() || (!original && !extended && !controlled_profile) ||
      !model.domain() || !model.contributions() ||
      !owner.owner_id || owner.epoch || owner.time != 0 || owner.velocity_time != 0 ||
      !owner.has_rotations || owner.reactions_valid || owner.reaction_base_epoch ||
      owner.reaction_time != 0 || owner.reaction_kick_dt != 0 ||
      owner.temporal_scheme != NodalTemporalScheme::StaggeredHalfKickStart ||
      owner.velocity_phase != NodalVelocityPhase::Collocated ||
      !tl::math::Finite(owner.fixed_dt) || owner.fixed_dt <= 0 ||
      !config.configuration_id || !config.qualification_id ||
      !shell_startup_detail::ValidStartup(config.startup, true, true) ||
      !config.cin_witness_count || config.cin_witness_count > NodalCinLimits{}.max_witnesses ||
      !DeclaredRigidScope(owner.rigid_groups, owner.node_count) ||
      owner.node_count != model.domain()->node_count()) {
    return {BatchStatus::InvalidInput, "Solid participant requires a fresh complete physical CIN profile"};
  }
  // Retained immutable size is checked before traversing material declarations.
  if (model.owned_payload_bytes() > config.limits.max_host_bytes) {
    return {BatchStatus::ResourceLimit, "Retained solid model exceeds host budget"};
  }
  Counts count{model.solid18().size(), model.solid24().size(), model.solid6z().size(),
      model.materials36().size(), model.materials42().size(), 0,
      model.solid18_law44().size(), model.solid18_law90().size(),
      model.materials44().size(), model.materials90().size()};
  if (count.material36 > config.limits.max_materials ||
      count.material42 > config.limits.max_materials - count.material36 ||
      count.material44 > config.limits.max_materials - count.material36 - count.material42 ||
      count.material90 > config.limits.max_materials - count.material36 - count.material42 - count.material44) {
    return {BatchStatus::ResourceLimit, "Solid material count exceeds admitted scope"};
  }
  const auto add_curve = [&](std::size_t points) {
    if (count.curve_points > config.limits.max_curve_points ||
        points > config.limits.max_curve_points - count.curve_points) return false;
    count.curve_points += points;
    return true;
  };
  for (const auto& material : model.materials36())
    if (!add_curve(material.value.curve.count))
      return {BatchStatus::ResourceLimit, "Solid owned curve pool exceeds admitted scope"};
  for (const auto& material : model.materials44()) {
    if (material.value.material.hardening == tl::material::law44::solid::HardeningKind::Analytic)
      ++count.analytic_material44;
    if (!add_curve(material.value.curve.count))
      return {BatchStatus::ResourceLimit, "Solid owned curve pool exceeds admitted scope"};
  }
  for (const auto& material : model.materials90())
    if (!add_curve(material.value.curve().count))
      return {BatchStatus::ResourceLimit, "Solid owned curve pool exceeds admitted scope"};
  if(controlled_profile) {
    const auto* selected=model.control_selection();
    if(!selected||selected->profile()!=control::Profile::SourceDeclared||!selected->controlled_count())
      return {BatchStatus::InvalidInput,"Source-controlled resident requires complete source-declared selection"};
    std::size_t capacity[controlled::Blocks]{};
    for(const auto& row:selected->parents())if(row.source.icontrol) {
      if(row.family==Family::Solid24)++count.controlled.h24;
      else if(row.family==Family::Solid18Law90)++count.controlled.foam;
      else return {BatchStatus::InvalidInput,"Controlled resident currently supports H24 and LAW90 only",row.family,row.family_index};
    }
    count.controlled.packets=selected->packets().size();count.controlled.members=selected->members().size();
    for(std::size_t p=0;p<selected->packets().size();++p) {
      const auto& packet=selected->packets()[p].source;if(!packet.icontrol)continue;
      if(packet.member_count>controlled::Threads)return {BatchStatus::InvalidInput,"Native NEL exceeds admitted controlled CTA width"};
      const auto worker=p%controlled::Blocks;
      if(packet.member_count>capacity[worker])capacity[worker]=packet.member_count;
    }
    for(unsigned n=0;n<controlled::Blocks;++n){count.controlled.worker_begin[n]=count.controlled.workers;count.controlled.workers+=capacity[n];}
  }
  if (!MakeLayout(count, config, output)) {
    return {BatchStatus::ResourceLimit, "Solid typed counts or complete arena exceed limits"};
  }
  return {};
}
} // namespace tl::fea::solids::batch_detail
