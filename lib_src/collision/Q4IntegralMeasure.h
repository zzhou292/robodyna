#pragma once

#include "Q4ContactBounds.h"

namespace tlfea::contact {
namespace q4_measure_detail {
TL_SURFACE_HD inline bool ExpandCertificate(Q4IntegralInterval ratio,Q4CertifiedIntegral* value) {
  Q4IntegralInterval truth;
  return q4_bounds::MultiplyPositive({value->lower,value->upper},ratio,&truth) &&
         q4_bounds::Certify(value->value,truth,value);
}
}  // namespace q4_measure_detail

// The raw integral and any immutable-area expansion have already certified
// finiteness. Apply the same unchanged absolute budgets at each adapter seam.
TL_SURFACE_HD inline bool WithinQ4IntegralBudgets(const Q4IntegrationResult& result,
                                                 const Q4IntegrationLimits& limits) {
  if (result.resultant.error > limits.force_error || result.potential.error > limits.energy_error) return false;
  for (const auto& force:result.force) if (force.error > limits.force_error) return false;
  return true;
}

// Expand a valid integral's continuum bounds from its defined FP64 reference
// measure to the corresponding exact-coordinate area enclosure. The rounded
// estimate/forces are unchanged, and may lie outside the truth enclosure; their
// reported errors cover that distance. This performs the original C4 measure
// arithmetic in the same order, without depending on an owner or wall class.
// Callers must recheck their declared N/J budgets after expansion. This must be
// applied exactly once to a raw integral; it is not a current-area update.
// Every caller result field is preserved on failure.
TL_SURFACE_HD inline bool ExpandQ4IntegralMeasure(double defined_area,Q4IntegralInterval exact_area,
                                                Q4IntegrationResult* output) {
  if (!output || !output->valid || !IsFinite(defined_area) || defined_area <= 0 ||
      !q4_bounds::Nonnegative(exact_area) || exact_area.lower <= 0 ||
      defined_area < exact_area.lower || defined_area > exact_area.upper) return false;
  Q4IntegralInterval ratio{1,1};
  if ((exact_area.lower != defined_area || exact_area.upper != defined_area) &&
      !q4_bounds::DividePositive(exact_area,defined_area,&ratio)) return false;
  auto staged=*output;
  for (auto& force:staged.force) if (!q4_measure_detail::ExpandCertificate(ratio,&force)) return false;
  if (!q4_measure_detail::ExpandCertificate(ratio,&staged.resultant) ||
      !q4_measure_detail::ExpandCertificate(ratio,&staged.potential) ||
      !q4_bounds::MultiplyPositive(staged.active_area,ratio,&staged.active_area)) return false;
  *output=staged; return true;
}
}  // namespace tlfea::contact
