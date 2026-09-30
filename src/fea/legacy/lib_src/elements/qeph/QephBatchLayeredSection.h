#pragma once
#include "QephLayeredJ2.h"
#include "QephLayeredLaw1.h"
#include "../ShellMixedSectionArenaLayout.h"
#include "../ShellBatchSectionAdvance.h"

namespace tl::fea::qeph::batch_detail {
// Borrow the actual accepted section and history; only the private target slab
// is written. The batch status/finalization gate precedes any publication.
TL_QEPH_HD inline Status EvaluatePlasticSection(const ReferenceData& reference,const History& old_shell,
    const PrescribedInterval& interval,shell_batch_plasticity_detail::DeviceStorage& storage,
    unsigned accepted_slab,std::size_t parent,ForceTrial& output) noexcept {
  if(reference.input.placement!=ShellReferencePlacement::Centered) return Status::kInvalidInput;
  const auto& old=storage.section[accepted_slab][parent];
  sections::ShellLayeredJ2Result section;
  const auto status=detail::EvaluateLayeredForceIntoTrial(reference,storage.parameters[parent],
      old_shell,interval,output,section,
      sections::LayeredJ2ForceAdapter{storage.parameters[parent],old.history});
  if(status!=Status::kSuccess) return status;
  return shell_batch_plasticity_detail::ProposedPlasticSection(old,section.history,section.diagnostics,
      old_shell.data().thickness,output.kinematics.area,storage.section[1u-accepted_slab][parent])?
      Status::kSuccess:Status::kNonfiniteResult;
}
// A single family-order law selects the already qualified adapter. Neither
// this dispatcher nor optional storage owns a clock or accepted selector.
TL_QEPH_HD inline Status EvaluateMixedSectionIntoTrial(const ReferenceData& reference,const History& old_shell,
    const PrescribedInterval& interval,shell_batch_plasticity_detail::MixedDeviceStorage& storage,
    unsigned accepted_slab,std::size_t parent,ForceTrial& output) noexcept {
  if(accepted_slab>1)return Status::kInvalidInput;
  const auto law=storage.law[parent];
  if(law==ShellSectionLaw::GlobalLaw1Npt0) {
    if(!storage.global_law1)return Status::kInvalidInput;
    return detail::EvaluateGlobalLaw1IntoTrial(storage.global_law1[parent],reference,old_shell,interval,output);
  }
  if(law==ShellSectionLaw::LayeredLaw1Nip3) {
    sections::ShellLayeredLaw1Result section;
    const auto status=detail::EvaluateLayeredLaw1IntoTrial(reference,storage.elastic_parameters[parent],
        old_shell,storage.elastic_section[accepted_slab][parent],interval,output,section);
    if(status!=Status::kSuccess) return status;
    storage.elastic_section[1u-accepted_slab][parent]=section.history;
    return Status::kSuccess;
  }
  if(law!=ShellSectionLaw::LayeredLaw44Nip3)return Status::kInvalidInput;
  return EvaluatePlasticSection(reference,old_shell,interval,storage.plastic,accepted_slab,parent,output);
}
// Existing host/value dispatcher keeps its all-output failure contract.
TL_QEPH_HD inline Status EvaluateMixedSection(const ReferenceData& reference,const History& old_shell,
    const PrescribedInterval& interval,shell_batch_plasticity_detail::MixedDeviceStorage& storage,
    unsigned accepted_slab,std::size_t parent,ForceTrial& output) noexcept {
  ForceTrial staged;
  const auto status=EvaluateMixedSectionIntoTrial(reference,old_shell,interval,storage,accepted_slab,parent,staged);
  if(status==Status::kSuccess) output=staged;
  return status;
}
} // namespace tl::fea::qeph::batch_detail
