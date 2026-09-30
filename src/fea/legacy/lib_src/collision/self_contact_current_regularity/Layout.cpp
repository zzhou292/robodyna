// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"

#include "../SurfaceMaterialMeasure.h"

#include <cstdint>

namespace tlfea::contact::current_regularity {
namespace {

using S = SelfContactCurrentRegularityStatus;

SelfContactCurrentRegularityReport Fail(
    S status, const char* message) noexcept {
  return {status, SIZE_MAX, SIZE_MAX, 0, 0, message};
}

bool ValidLimits(SelfContactCurrentRegularityLimits value) noexcept {
  const auto hard = SelfContactCurrentRegularityLimits::Vehicle();
  return value.max_parents && value.max_parents <= hard.max_parents &&
      value.max_facets && value.max_facets <= hard.max_facets &&
      value.max_host_bytes && value.max_host_bytes <= hard.max_host_bytes;
}

}  // namespace

SelfContactCurrentRegularityReport MakeLayout(
    const SelfContactActiveUseBinding& binding,
    SelfContactCurrentRegularityLimits limits,
    std::size_t implementation_bytes, Layout& output) noexcept {
  if (!binding.prepared() || !binding.facets() ||
      !binding.facets()->prepared() || !binding.facets()->surface() ||
      !binding.facets()->surface()->prepared() ||
      !binding.facets()->surface()->physical() ||
      !binding.facets()->surface()->physical()->domain())
    return Fail(S::InvalidInput,
        "Prepared active-use/facet/S0 authority is required");
  if (!ValidLimits(limits))
    return Fail(S::ResourceLimit,
        "Current-regularity limits exceed the fixed hard profile");

  Layout next;
  const auto active = binding.forecast();
  next.forecast.parents = active.parents;
  next.forecast.facets = active.facets;
  if (!next.forecast.parents || !next.forecast.facets ||
      next.forecast.parents != binding.parents().size() ||
      next.forecast.facets != binding.facet_uses().size() ||
      next.forecast.parents > UINT32_MAX ||
      active.node_roles > UINT32_MAX)
    return Fail(S::IdentityMismatch,
        "Active-use parent/facet/node inventory is incomplete");
  if (next.forecast.parents > limits.max_parents ||
      next.forecast.facets > limits.max_facets)
    return Fail(S::ResourceLimit,
        "Complete current parent/facet inventory exceeds a count cap");
  if (next.forecast.parents >
      SIZE_MAX / (2 * sizeof(SelfContactCurrentParentResult)))
    return Fail(S::ResourceLimit,
        "Checked current publication count overflowed");
  next.forecast.publication_records = 2 * next.forecast.parents;
  // Runtime facet geometry is one fixed-size stack value, never a
  // facet-count-sized retained staging array.
  next.forecast.facet_staging_records = 0;
  const auto fixed = binding.facets()->forecast();
  if (!fixed.q4_template_facets || !fixed.t3_template_facets)
    return Fail(S::IdentityMismatch,
        "Fixed-facet template inventory is incomplete");

  tl::util::BoundedArenaLayout arena(limits.max_host_bytes);
  if (!arena.Append<SelfContactCurrentParentResult>(
          next.forecast.parents, next.first_results) ||
      !arena.Append<SelfContactCurrentParentResult>(
          next.forecast.parents, next.second_results) ||
      !arena.Append<FacetTemplate>(
          fixed.q4_template_facets, next.q4_templates) ||
      !arena.Append<FacetTemplate>(
          fixed.t3_template_facets, next.t3_templates))
    return Fail(S::ResourceLimit,
        "Preallocated current results/templates exceed the byte cap");
  next.forecast.arena_bytes = arena.bytes();

  const auto retained = binding.forecast().owned_payload_bytes;
  if (retained < sizeof(SelfContactActiveUseBinding))
    return Fail(S::IdentityMismatch,
        "Retained active-use byte forecast is invalid");
  next.forecast.retained_active_use_bytes =
      retained - sizeof(SelfContactActiveUseBinding);

  tl::util::BoundedArenaLayout budget(limits.max_host_bytes);
  tl::util::ArenaRegion ignored;
  if (implementation_bytes > SIZE_MAX-sizeof(SelfContactCurrentRegularity)-64 ||
      !budget.Append<std::byte>(
          sizeof(SelfContactCurrentRegularity)+implementation_bytes+64,
          ignored) ||
      !budget.Append<std::byte>(
          next.forecast.retained_active_use_bytes, ignored) ||
      !budget.Append<std::byte>(next.forecast.arena_bytes, ignored))
    return Fail(S::ResourceLimit,
        "Current binding, retained source and arena exceed the byte cap");
  next.forecast.owned_payload_bytes = budget.bytes();
  constexpr std::size_t scratch =
      sizeof(Layout) + sizeof(FixedContactFacetReadCursor) +
      sizeof(Q4MaterialMeasure) + sizeof(T3MaterialMeasure) +
      sizeof(SelfContactCurrentRegularitySummary) +
      sizeof(WeightedSurfacePoint) + 3*sizeof(Vec3) +
      sizeof(SelfContactCurrentFacetWitness);
  if (!budget.Append<std::byte>(scratch, ignored))
    return Fail(S::ResourceLimit,
        "Typed current geometry startup/query scratch exceeds the byte cap");
  next.forecast.startup_payload_bytes = budget.bytes();
  output = next;
  return {};
}

}  // namespace tlfea::contact::current_regularity
