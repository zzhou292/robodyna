// Generated from complete authenticated bodies; no numerical edits.
#pragma once
#include "lib_src/elements/t3/T3Batch.h"
#include "lib_src/elements/t3/T3OnePointHistory.h"
#include "lib_src/elements/t3/mapped/Result.h"
#include "lib_src/elements/failure/ShellFailureReadback.h"
namespace t3_readback_test::candidate {
using namespace tl::fea;
using namespace tl::fea::t3;
template<class State, class ReadResults>
BatchReport ReadOnePoint(State& state, unsigned slab, double time,
    std::uint64_t epoch, ReadResults read_results) {
  if (!state.plasticity || !state.plasticity->one_point_sections() || !state.joined_binding || slab > 1) {
    return {BatchStatus::InvalidInput, "One-point readback has no complete T3 source scope"};
  }
  const auto* catalog = state.plasticity->section_catalog();
  if (!catalog) return {BatchStatus::InvalidInput, "One-point readback catalog is unavailable"};
  const auto report = read_results();
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
template<class State> BatchReport Activity(State& state, unsigned slab, double time, std::uint64_t epoch) {
  return shell_batch_plasticity_detail::ReadFailure(state, slab, time, [&](unsigned selected) {
    const auto report = state.ValidateMappedSections(selected);
    if (report.status != BatchStatus::Success || !state.plasticity->one_point_sections()) return report;
    if (!state.physical) return state.ValidateOnePointReadback(selected, time, epoch);

    // Successful mapped validation just copied and validated this complete slab.
    // Only const host role checks intervened, and batch calls are serialized.
    // Reuse is confined to this call: no retained validity bit or staged API.
    // Preserve the old second pending-error check before one-point validation.
    return ReadOnePoint(state, selected, time, epoch, [&] { return state.PendingError(); });
  });
}
}
