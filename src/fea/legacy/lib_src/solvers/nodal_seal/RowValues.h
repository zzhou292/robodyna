#pragma once
#include "../ExplicitStepStability.h"

namespace tl::fea::nodal_seal {
// Maxima have no summation roundoff. Equal positive values retain the first
// source node; zero never replaces the serial finalizer's initial positive zero.
struct RowSummary {
  double stiffness;
  double damping;
  std::uint32_t stiffness_node;
  std::uint32_t damping_node;
  std::uint32_t invalid_node;
};
static_assert(sizeof(RowSummary) == 32);
TL_SURFACE_HD inline RowSummary EmptyRows() { return {0, 0, 0, 0, UINT32_MAX}; }

TL_SURFACE_HD inline void SelectMaximum(double value, std::uint32_t node,
    double& maximum, std::uint32_t& first) {
  if (value > maximum || (value > 0 && value == maximum && node < first)) {
    maximum = value;
    first = node;
  }
}
TL_SURFACE_HD inline void IncludeRow(RowSummary& out, double stiffness,
    double damping, std::uint32_t node) {
  if (!stability::IsFinite(stiffness) || stiffness < 0 ||
      !stability::IsFinite(damping) || damping < 0) {
    if (node < out.invalid_node) out.invalid_node = node;
    return;
  }
  SelectMaximum(stiffness, node, out.stiffness, out.stiffness_node);
  SelectMaximum(damping, node, out.damping, out.damping_node);
}
TL_SURFACE_HD inline void CombineRows(RowSummary& out, const RowSummary& next) {
  SelectMaximum(next.stiffness, next.stiffness_node, out.stiffness, out.stiffness_node);
  SelectMaximum(next.damping, next.damping_node, out.damping, out.damping_node);
  if (next.invalid_node < out.invalid_node) out.invalid_node = next.invalid_node;
}

// Read-only eligibility: an unexpected layout or invalid header must reach the
// complete legacy finalizer, which owns cleared outputs and failure flags.
TL_SURFACE_HD inline bool CanReduceRows(const stability::RowBounds& rows,
    const double* scratch, std::uint32_t n, double safety, double minimum_dt, double h) {
  return scratch && n && rows.valid && rows.initialized && !rows.sealed &&
      rows.node_count == n && rows.capacity >= n &&
      rows.stiffness == scratch + std::size_t(6) * n &&
      rows.damping == scratch + std::size_t(7) * n &&
      stability::IsFinite(safety) && safety > 0 && safety < 1 &&
      stability::IsFinite(minimum_dt) && minimum_dt > 0 &&
      stability::IsFinite(h) && h >= minimum_dt;
}

// Private caller has validated every row in this attempt. Invalid rows still
// use FinalizeRows, including its original full prefix and partial-state rules.
TL_SURFACE_HD inline stability::Status FinalizeReducedRows(stability::RowBounds* rows,
    double safety, double minimum_dt, double h, const RowSummary& summary,
    stability::StepLimit* out) {
  if (summary.invalid_node != UINT32_MAX)
    return stability::FinalizeRows(rows, safety, minimum_dt, h, out);
  stability::StepLimit result;
  const auto status = stability::detail::BeginFinalizeRows(rows, safety, minimum_dt, h, out, result);
  if (status != stability::Status::kOk) return status;
  result.stiffness_bound = summary.stiffness;
  result.damping_bound = summary.damping;
  result.stiffness_node = summary.stiffness_node;
  result.damping_node = summary.damping_node;
  return stability::detail::CompleteFinalizeRows(rows, safety, minimum_dt, result, out);
}
} // namespace tl::fea::nodal_seal
