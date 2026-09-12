// Generated from complete authenticated bodies; no numerical edits.
#pragma once
#include "lib_src/elements/t3/T3Batch.h"
#include "lib_src/elements/t3/T3OnePointHistory.h"
#include "lib_src/elements/t3/mapped/Result.h"
#include "lib_src/elements/failure/ShellFailureReadback.h"
namespace t3_readback_test::serial {
using namespace tl::fea;
using namespace tl::fea::t3;
template<class State> BatchReport OnePoint(State& state, unsigned slab, double time, std::uint64_t epoch) {
  if (!state.plasticity || !state.plasticity->one_point_sections() || !state.joined_binding || slab > 1) {
    return {BatchStatus::InvalidInput, "One-point readback has no complete T3 source scope"};
  }
  const auto* catalog = state.plasticity->section_catalog();
  if (!catalog) return {BatchStatus::InvalidInput, "One-point readback catalog is unavailable"};
  const auto report = state.ReadResults(&state.storage->slab[slab]);
  if (report.status != BatchStatus::Success) return report;
  const auto* sections = state.plasticity->section_staging();
  for (std::size_t e = 0; e < state.config.element_count; ++e) {
    const auto* point = sections[e].one_point();
    if (!point) continue;
    const auto& history = state.staging[e].proposed_history;
    const auto& reference = state.joined_binding->t3_reference(e);
    const auto& saved = point->point.saved;
    sections::PointParameters parameters;
    if (!catalog->Parameters(ShellBindingFamily::T3, e, &parameters) ||
        !history.matches_reference(reference) || history.stamp().time != time ||
        history.stamp().sample_index != epoch ||
        !detail::SameHistoryBits(history.data().thickness, point->point.reported_thickness_m) ||
        !one_point_detail::ValidValues({history.data(), saved, point->point.failure.history,
            point->cumulative_plastic_work_J}, parameters, time)) {
      return {BatchStatus::NonfiniteResult, "One-point state differs from its actual shell slab/history identity",
              static_cast<std::uint32_t>(e)};
    }
  }
  return {BatchStatus::Success, "OK"};
}
template<class State> BatchReport ValidateMappedResults(State& state, unsigned slab) {
  if (!state.physical) return {BatchStatus::Success,"OK"};
  const bool accepted=slab==state.AcceptedSlabIndex();
  const auto epoch=state.accepted_stamp.epoch+(accepted?0:1);
  const auto time=state.accepted_stamp.time+(accepted?0:state.config.owner.fixed_dt);
  for (std::size_t parent=0;parent<state.config.element_count;++parent) {
    ShellSectionLaw law=ShellSectionLaw::Unspecified;
    if (!state.physical->catalog()->Law(ShellBindingFamily::T3,parent,&law) ||
        !mapped::ValidResult(state.physical->shells()->t3_reference(parent),state.staging[parent],time,epoch,
            law==ShellSectionLaw::RigidSkin)) {
      return {BatchStatus::NonfiniteResult,"Mapped T3 force cache differs from its source/endpoint role",
          static_cast<std::uint32_t>(parent)};
    }
  }
  return {BatchStatus::Success,"OK"};
}
template<class State> BatchReport ValidateMappedSections(State& state, unsigned slab) {
  if (!state.physical) return {BatchStatus::Success,"OK"};
  const auto report=state.ReadResults(&state.storage->slab[slab]);
  if (report.status!=BatchStatus::Success) return report;
  const auto* sections=state.plasticity->section_staging();
  const auto* failure=state.plasticity->failure_staging();
  if (!sections || !failure) return {BatchStatus::InvalidInput,"Mapped T3 typed section shape is missing"};
  for (std::size_t parent=0;parent<state.config.element_count;++parent) {
    ShellSectionLaw law=ShellSectionLaw::Unspecified;
    if (!state.physical->catalog()->Law(ShellBindingFamily::T3,parent,&law) || sections[parent].law()!=law) {
      return {BatchStatus::NonfiniteResult,"Mapped T3 typed section role differs",static_cast<std::uint32_t>(parent)};
    }
    // True NIP1 failure belongs to the point payload, not the reserved NIP3 slot.
    const bool active=sections[parent].one_point()?sections[parent].one_point()->point.failure.history.point_active:
        failure[parent].active;
    if (state.staging[parent].proposed_history.data().active!=(active?1:0)) {
      return {BatchStatus::NonfiniteResult,"Mapped T3 force and failure activity differ",static_cast<std::uint32_t>(parent)};
    }
  }
  return {BatchStatus::Success,"OK"};
}
template<class State> BatchReport Activity(State& state, unsigned slab, double time, std::uint64_t epoch) {
  const auto report = shell_batch_plasticity_detail::ReadFailure(state,
      slab, time);
  if (report.status != BatchStatus::Success) return report;
  if (state.plasticity->one_point_sections()) {
    const auto checked = state.ValidateOnePointReadback(slab,
        time, epoch);
    if (checked.status != BatchStatus::Success) return checked;
  }
  return report;
}
}
