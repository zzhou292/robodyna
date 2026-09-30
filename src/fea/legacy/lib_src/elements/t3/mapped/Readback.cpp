// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../T3BatchStorage.h"
#include "Result.h"
#include "../../ShellPhysicalOutputRanges.h"
#include "../../ShellFormulationOutputRanges.h"

namespace tl::fea::t3 {
bool T3Batch::Impl::OutputDisjoint(const void* output,std::size_t bytes) const noexcept {
  if (!physical) return true; // Preserve the legacy output contract.
  using trial_identity::Disjoint;
  if (!Disjoint(output,bytes,this,sizeof(*this)) ||
      !Disjoint(output,bytes,staging.data(),config.element_count*sizeof(ForceTrial)) ||
      !Disjoint(output,bytes,activity_staging.data(),activity_staging.size()) ||
      !Disjoint(output,bytes,activity_roles.data(),activity_roles.size()) ||
      !Disjoint(output,bytes,activity_failure.data(),activity_failure.size()) ||
      !shell_physical_owner::OutputDisjoint(*physical,output,bytes)) return false;
  if (!plasticity || !joined_binding || !plasticity->failure_binding()) return false;
  const ShellFormulationScope source{&*joined_binding,plasticity->section_catalog(),
      plasticity->failure_binding(),nullptr};
  return Disjoint(output,bytes,plasticity.get(),sizeof(*plasticity)) &&
      shell_formulation_detail::OutputDisjoint(source,output,bytes);
}
BatchReport T3Batch::Impl::ValidateMappedResults(unsigned slab) const noexcept {
  if (!physical) return {BatchStatus::Success,"OK"};
  const bool accepted=slab==AcceptedSlabIndex();
  const auto epoch=accepted_stamp.epoch+(accepted?0:1);
  const auto time=accepted_stamp.time+(accepted?0:config.owner.fixed_dt);
  for (std::size_t parent=0;parent<config.element_count;++parent) {
    ShellSectionLaw law=ShellSectionLaw::Unspecified;
    if (!physical->catalog()->Law(ShellBindingFamily::T3,parent,&law) ||
        !mapped::ValidResult(physical->shells()->t3_reference(parent),staging[parent],time,epoch,
            law==ShellSectionLaw::RigidSkin)) {
      return {BatchStatus::NonfiniteResult,"Mapped T3 force cache differs from its source/endpoint role",
          static_cast<std::uint32_t>(parent)};
    }
  }
  return {BatchStatus::Success,"OK"};
}
BatchReport T3Batch::Impl::ValidateMappedSections(unsigned slab) {
  if (!physical) return {BatchStatus::Success,"OK"};
  const auto report=ReadResults(&storage->slab[slab]);
  if (report.status!=BatchStatus::Success) return report;
  const auto* sections=plasticity->section_staging();
  const auto* failure=plasticity->failure_staging();
  if (!sections || !failure) return {BatchStatus::InvalidInput,"Mapped T3 typed section shape is missing"};
  for (std::size_t parent=0;parent<config.element_count;++parent) {
    ShellSectionLaw law=ShellSectionLaw::Unspecified;
    if (!physical->catalog()->Law(ShellBindingFamily::T3,parent,&law) || sections[parent].law()!=law) {
      return {BatchStatus::NonfiniteResult,"Mapped T3 typed section role differs",static_cast<std::uint32_t>(parent)};
    }
    // True NIP1 failure belongs to the point payload, not the reserved NIP3 slot.
    const bool active=sections[parent].one_point()?sections[parent].one_point()->point.failure.history.point_active:
        failure[parent].active;
    if (staging[parent].proposed_history.data().active!=(active?1:0)) {
      return {BatchStatus::NonfiniteResult,"Mapped T3 force and failure activity differ",static_cast<std::uint32_t>(parent)};
    }
  }
  return {BatchStatus::Success,"OK"};
}
} // namespace tl::fea::t3
