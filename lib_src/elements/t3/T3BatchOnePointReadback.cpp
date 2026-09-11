#include "T3BatchStorage.h"
#include "T3OnePointHistory.h"

namespace tl::fea::t3 {
BatchReport T3Batch::Impl::ValidateOnePointReadback(unsigned slab, double time, std::uint64_t epoch) {
  if (!plasticity || !plasticity->one_point_sections() || !joined_binding || slab > 1) {
    return {BatchStatus::InvalidInput, "One-point readback has no complete T3 source scope"};
  }
  const auto* catalog = plasticity->section_catalog();
  if (!catalog) return {BatchStatus::InvalidInput, "One-point readback catalog is unavailable"};
  const auto report = ReadResults(&storage->slab[slab]);
  if (report.status != BatchStatus::Success) return report;
  const auto* sections = plasticity->section_staging();
  for (std::size_t e = 0; e < config.element_count; ++e) {
    const auto* point = sections[e].one_point();
    if (!point) continue;
    const auto& history = staging[e].proposed_history;
    const auto& reference = joined_binding->t3_reference(e);
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
} // namespace tl::fea::t3
