#pragma once
#include "QephLayeredJ2.h"
#include "QephLayeredLaw1.h"
#include "../ShellMixedSectionArenaLayout.h"
#include "../ShellBatchSectionAdvance.h"

namespace tl::fea::qeph::batch_detail {
// A single family-order law selects the already qualified adapter. Neither
// this dispatcher nor optional storage owns a clock or accepted selector.
TL_QEPH_HD inline Status EvaluateMixedSection(const ReferenceData& reference,const History& old_shell,
    const PrescribedInterval& interval,shell_batch_plasticity_detail::MixedDeviceStorage& storage,
    unsigned accepted_slab,std::size_t parent,ForceTrial& output) noexcept {
  if(accepted_slab>1)return Status::kInvalidInput;
  const auto law=storage.law[parent];
  if(law==ShellSectionLaw::LayeredLaw1Nip3) {
    const LayeredLaw1History base{old_shell,storage.elastic_section[accepted_slab][parent]};
    LayeredLaw1ForceTrial next;
    const auto status=EvaluateLayeredLaw1Force(reference,storage.elastic_parameters[parent],base,interval,next);
    if(status!=Status::kSuccess)return status;
    output=next.force;storage.elastic_section[1u-accepted_slab][parent]=next.proposed_section;
    return Status::kSuccess;
  }
  if(law!=ShellSectionLaw::LayeredLaw44Nip3)return Status::kInvalidInput;
  auto& plastic=storage.plastic;const auto& old=plastic.section[accepted_slab][parent];
  const LayeredJ2History base{old_shell,old.history};LayeredJ2ForceTrial next;
  const auto status=EvaluateLayeredJ2Force(reference,plastic.parameters[parent],base,interval,next);
  if(status!=Status::kSuccess)return status;
  ShellBatchSectionState section;
  if(!shell_batch_plasticity_detail::ProposedPlasticSection(old,next.proposed_section,next.section_diagnostics,
      base.shell.data().thickness,next.force.kinematics.area,section))return Status::kNonfiniteResult;
  output=next.force;plastic.section[1u-accepted_slab][parent]=section;return Status::kSuccess;
}
} // namespace tl::fea::qeph::batch_detail
