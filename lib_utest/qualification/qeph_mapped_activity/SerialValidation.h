// Complete baseline validation bodies; only explicit State qualification changed.
#pragma once
#include "lib_src/elements/qeph/mapped/Result.h"
namespace qeph_activity_test::serial {
using namespace tl::fea;
using namespace tl::fea::qeph;
template<class State>
BatchReport ValidateMappedResults(const State& state,unsigned slab) noexcept {
  if (!state.physical) return {BatchStatus::Success,"OK"};
  const bool accepted=slab==state.AcceptedSlabIndex();
  const auto epoch=state.accepted_stamp.epoch+(accepted?0:1);
  const auto time=state.accepted_stamp.time+(accepted?0:state.config.owner.fixed_dt);
  for (std::size_t parent=0;parent<state.config.element_count;++parent) {
    ShellSectionLaw law=ShellSectionLaw::Unspecified;
    if (!state.physical->catalog()->Law(ShellBindingFamily::Qeph,parent,&law) ||
        !mapped::ValidResult(state.physical->shells()->qeph_reference(parent),state.staging[parent],time,epoch,
            law==ShellSectionLaw::RigidSkin)) {
      return {BatchStatus::NonfiniteResult,"Mapped Qeph force cache differs from its source/endpoint role",
          static_cast<std::uint32_t>(parent)};
    }
  }
  return {BatchStatus::Success,"OK"};
}
template<class State>
BatchReport ValidateMappedSections(State& state,unsigned slab) {
  if (!state.physical) return {BatchStatus::Success,"OK"};
  const auto report=state.ReadResults(&state.storage->slab[slab]);
  if (report.status!=BatchStatus::Success) return report;
  const auto* sections=state.plasticity->section_staging();
  const auto* failure=state.plasticity->failure_staging();
  if (!sections || !failure) return {BatchStatus::InvalidInput,"Mapped Qeph typed section shape is missing"};
  for (std::size_t parent=0;parent<state.config.element_count;++parent) {
    ShellSectionLaw law=ShellSectionLaw::Unspecified;
    if (!state.physical->catalog()->Law(ShellBindingFamily::Qeph,parent,&law) || sections[parent].law()!=law) {
      return {BatchStatus::NonfiniteResult,"Mapped Qeph typed section role differs",static_cast<std::uint32_t>(parent)};
    }
    // True NIP1 failure belongs to the point payload, not the reserved NIP3 slot.
    const bool active=sections[parent].one_point()?sections[parent].one_point()->point.failure.history.point_active:
        failure[parent].active;
    if (state.staging[parent].proposed_history.data().active!=(active?1:0)) {
      return {BatchStatus::NonfiniteResult,"Mapped Qeph force and failure activity differ",static_cast<std::uint32_t>(parent)};
    }
  }
  return {BatchStatus::Success,"OK"};
}
}
