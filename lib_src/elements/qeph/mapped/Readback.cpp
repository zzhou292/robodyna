// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../QephBatchStorage.h"
#include "Result.h"
#include "ActivityValues.h"
#include "../../ShellPhysicalOutputRanges.h"
#include "../../ShellFormulationOutputRanges.h"

namespace tl::fea::qeph {
bool QephBatch::Impl::OutputDisjoint(const void* output,std::size_t bytes) const noexcept {
  if (!physical) return true; // Preserve the legacy output contract.
  using trial_identity::Disjoint;
  if (!Disjoint(output,bytes,this,sizeof(*this)) ||
      !Disjoint(output,bytes,staging.data(),config.element_count*sizeof(ForceTrial)) ||
      !Disjoint(output,bytes,activity_staging.data(),activity_staging.size()) ||
      !Disjoint(output,bytes,failure_activity_staging.data(),failure_activity_staging.size()) ||
      !shell_physical_owner::OutputDisjoint(*physical,output,bytes)) return false;
  if (!plasticity || !joined_binding || !plasticity->failure_binding()) return false;
  const ShellFormulationScope source{&*joined_binding,plasticity->section_catalog(),
      plasticity->failure_binding(),nullptr};
  return Disjoint(output,bytes,plasticity.get(),sizeof(*plasticity)) &&
      shell_formulation_detail::OutputDisjoint(source,output,bytes);
}
BatchReport QephBatch::Impl::ValidateMappedResults(unsigned slab) const noexcept {
  if (!physical) return {BatchStatus::Success,"OK"};
  const bool accepted=slab==AcceptedSlabIndex();
  const auto epoch=accepted_stamp.epoch+(accepted?0:1);
  const auto time=accepted_stamp.time+(accepted?0:config.owner.fixed_dt);
  for (std::size_t parent=0;parent<config.element_count;++parent) {
    ShellSectionLaw law=ShellSectionLaw::Unspecified;
    if (!physical->catalog()->Law(ShellBindingFamily::Qeph,parent,&law) ||
        !mapped::ValidResult(physical->shells()->qeph_reference(parent),staging[parent],time,epoch,
            law==ShellSectionLaw::RigidSkin)) {
      return {BatchStatus::NonfiniteResult,"Mapped Qeph force cache differs from its source/endpoint role",
          static_cast<std::uint32_t>(parent)};
    }
  }
  return {BatchStatus::Success,"OK"};
}
BatchReport QephBatch::Impl::ValidateMappedSections(unsigned slab) {
  if (!physical) return {BatchStatus::Success,"OK"};
  const auto report=ReadResults(&storage->slab[slab]);
  if (report.status!=BatchStatus::Success) return report;
  return mapped::ValidateSectionActivity(*physical->catalog(),plasticity->section_staging(),
      plasticity->failure_staging(),config.element_count,
      [this](std::size_t parent) { return staging[parent].proposed_history.data().active; });
}
} // namespace tl::fea::qeph
