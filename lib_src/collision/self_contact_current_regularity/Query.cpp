// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"

#include "../FixedTriangleFeatureDiscovery.h"
#include "../Q4SurfaceMapping.h"
#include "../SurfaceContactGeometry.h"
#include "../SurfaceMaterialMeasure.h"

#include <algorithm>
#include <cfloat>
#include <cmath>

namespace tlfea::contact::current_regularity {
namespace {

using S = SelfContactCurrentRegularityStatus;

SelfContactCurrentRegularityReport Fail(
    S status, const SelfContactCurrentRegularityForecast& forecast,
    const char* message, std::size_t parent = SIZE_MAX,
    std::size_t facet = SIZE_MAX) noexcept {
  return {status, parent, facet,
          forecast.parents, forecast.facets, message};
}

S ParentFailure(SurfaceMeasureStatus status) noexcept {
  if (status == SurfaceMeasureStatus::UnresolvedGeometry)
    return S::ParentGeometryUnresolved;
  if (status == SurfaceMeasureStatus::Unrepresentable)
    return S::Unrepresentable;
  return S::InvalidInput;
}

SelfContactCurrentRegularityReport ParentChart(
    const SelfContactParentUse& parent, VectorView positions,
    const SelfContactCurrentRegularityForecast& forecast,
    SelfContactCurrentParentResult& result) noexcept {
  if (parent.arity == 4) {
    SurfaceQ4 source;
    source.feature_id = source.parent_element_id =
        parent.source.source_parent_id;
    for (unsigned i = 0; i < 4; ++i) source.nodes[i] = parent.nodes[i];
    Q4MaterialMeasure measure;
    const auto status =
        PrepareQ4MaterialMeasure(positions, source, &measure);
    if (status != SurfaceMeasureStatus::Ok)
      return Fail(ParentFailure(status), forecast,
          "Current Q4 fixed-direction regularity chart rejected",
          result.binding_parent);
    result.chart =
        SelfContactCurrentChartStatus::CertifiedQ4FixedDirection;
    result.chart_direction = measure.direction();
    result.current_area_enclosure_m2 = measure.area_enclosure();
  } else {
    SurfaceTriangle source;
    source.feature_id = source.parent_element_id =
        parent.source.source_parent_id;
    source.interpolation = SurfaceInterpolation::kLinearTriangle;
    for (unsigned i = 0; i < 3; ++i) source.nodes[i] = parent.nodes[i];
    T3MaterialMeasure measure;
    const auto status =
        PrepareT3MaterialMeasure(positions, source, &measure);
    if (status != SurfaceMeasureStatus::Ok)
      return Fail(ParentFailure(status), forecast,
          "Current T3 native regularity chart rejected",
          result.binding_parent);
    const Vec3 normal = geometry_detail::Cross(
        Subtract(measure.position(1), measure.position(0)),
        Subtract(measure.position(2), measure.position(0)));
    const double scale = geometry_detail::MaxAbs(normal);
    if (!IsFinite(normal) || !(scale > 0))
      return Fail(S::Unrepresentable, forecast,
          "Current T3 chart direction is unrepresentable",
          result.binding_parent);
    result.chart_direction = geometry_detail::Divide(normal, scale);
    result.chart =
        SelfContactCurrentChartStatus::CertifiedT3Native;
    result.current_area_enclosure_m2 = measure.area_enclosure();
  }
  if (!q4_bounds::Nonnegative(result.current_area_enclosure_m2) ||
      !(result.current_area_enclosure_m2.lower > 0))
    return Fail(S::Unrepresentable, forecast,
        "Current parent area enclosure is not strictly positive",
        result.binding_parent);
  return {};
}

SelfContactCurrentRegularityReport FacetFailure(
    SelfContactCurrentFacetStatus status,
    const SelfContactCurrentRegularityForecast& forecast,
    std::size_t parent, std::size_t facet) noexcept {
  if (status == SelfContactCurrentFacetStatus::Degenerate)
    return Fail(S::FacetDegenerate, forecast,
        "Represented current facet is degenerate or below quality boundary",
        parent, facet);
  if (status == SelfContactCurrentFacetStatus::Reversed)
    return Fail(S::FacetOrientationMismatch, forecast,
        "Represented current facet orientation differs from parent chart",
        parent, facet);
  if (status == SelfContactCurrentFacetStatus::Unrepresentable)
    return Fail(S::Unrepresentable, forecast,
        "Represented current facet certificate is unrepresentable",
        parent, facet);
  return Fail(S::InvalidInput, forecast,
      "Represented current facet input is invalid", parent, facet);
}

}  // namespace

SelfContactCurrentRegularityReport RunQuery(
    const SelfContactActiveUseBinding& binding,
    const SelfContactCurrentRegularityForecast& forecast,
    VectorView positions, SelfContactActivityView activity,
    SelfContactCurrentParentResult* staging,
    CurrentFixedTriangle* facet_staging,
    SelfContactCurrentRegularitySummary* summary_output) noexcept {
  if (!staging || !facet_staging || !summary_output ||
      !activity.base || !activity.current ||
      activity.parent_count != forecast.parents)
    return Fail(S::InvalidInput, forecast,
        "Current activity/result staging is incomplete");
  // Validate the complete activity before any staging write.  In particular,
  // a 0->1 transition is never interpreted as reactivation.
  for (std::size_t p = 0; p < forecast.parents; ++p)
    if (activity.base[p] > 1 ||
        activity.current[p] > activity.base[p])
      return Fail(S::InvalidInput, forecast,
          "Current activity is nonbinary or attempts reactivation", p);

  SelfContactCurrentRegularitySummary summary;
  summary.parents = forecast.parents;
  summary.facets = forecast.facets;
  const auto* fixed = binding.facets();
  const auto source_instance_id =
      fixed->surface()->physical()->domain()->source_instance_id();

  for (std::size_t p = 0; p < forecast.parents; ++p) {
    const auto& parent = binding.parents()[p];
    auto& result = staging[p];
    result = {};
    result.source_instance_id = source_instance_id;
    result.source_eid = parent.source.source_parent_id;
    result.binding_parent = p;
    result.surface_parent = parent.surface_parent;
    result.arity = parent.arity;
    result.level = parent.level;
    result.facet_count = parent.facet_count;
    if (!activity.base[p]) {
      result.state =
          SelfContactCurrentParentState::LongInactiveSkipped;
      result.chart =
          SelfContactCurrentChartStatus::SkippedLongInactive;
      ++summary.skipped_parents;
      continue;
    }
    result.state = activity.current[p]
        ? SelfContactCurrentParentState::Active
        : SelfContactCurrentParentState::Removing;
    summary.active_parents += activity.current[p] != 0;
    summary.removing_parents += activity.current[p] == 0;

    auto report = ParentChart(parent, positions, forecast, result);
    if (report.status != S::Ok) return report;
    result.geometry_evaluated = true;
    const auto approximation_status =
        fixed->Approximation(parent.surface_parent, positions,
                             &result.approximation);
    if (approximation_status != Status::kOk)
      return Fail(approximation_status == Status::kInvalidArgument
                      ? S::InvalidInput : S::Unrepresentable,
          forecast, "Current fixed-facet approximation could not be certified",
          p);

    bool have_facet = false;
    for (std::uint32_t local = 0; local < parent.facet_count; ++local) {
      const auto global = std::size_t(parent.facet_offset) + local;
      if (global >= forecast.facets)
        return Fail(S::IdentityMismatch, forecast,
            "Current parent facet range exceeds retained inventory",
            p, local);
      FixedContactFacet descriptor;
      if (fixed->Describe(parent.surface_parent, local, &descriptor).status !=
              FixedContactFacetStatus::Ok ||
          descriptor.source_instance_id != source_instance_id ||
          descriptor.source.source_parent_id != result.source_eid ||
          descriptor.level != result.level ||
          descriptor.local_facet != local)
        return Fail(S::IdentityMismatch, forecast,
            "Current facet replay differs from initialized authority",
            p, local);
      auto& triangle = facet_staging[global];
      const auto evaluation =
          EvaluateCurrentFixedTriangle(descriptor, positions, &triangle);
      if (evaluation != Status::kOk)
        return Fail(evaluation == Status::kInvalidArgument
                        ? S::InvalidInput : S::Unrepresentable,
            forecast, "Current weighted facet evaluation failed",
            p, local);
      SelfContactCurrentFacetWitness witness;
      const auto facet_status = EvaluateCurrentFacetRegularity(
          triangle.vertices, result.chart_direction, &witness);
      if (facet_status != SelfContactCurrentFacetStatus::Ok)
        return FacetFailure(facet_status, forecast, p, local);

      if (!have_facet ||
          witness.double_area_m2.lower <
              result.minimum_area_witness.double_area_m2.lower) {
        result.minimum_area_witness = witness;
        result.minimum_area_local_facet = local;
      }
      if (!have_facet ||
          witness.scaled_jacobian_quality <
              result.minimum_scaled_jacobian_quality) {
        result.minimum_scaled_jacobian_quality =
            witness.scaled_jacobian_quality;
        result.minimum_quality_local_facet = local;
      }
      if (!have_facet ||
          witness.directed_chart_measure_m2.lower <
              result.minimum_directed_chart_measure_m2.lower) {
        result.minimum_directed_chart_measure_m2 =
            witness.directed_chart_measure_m2;
        result.minimum_directed_local_facet = local;
      }
      have_facet = true;
      ++result.facets_evaluated;
      ++summary.facets_evaluated;
    }
    if (!have_facet || result.facets_evaluated != result.facet_count)
      return Fail(S::IdentityMismatch, forecast,
          "Current parent did not evaluate every fixed facet", p);

    if (!q4_bounds::Add(
            summary.certified_current_area_enclosure_m2,
            result.current_area_enclosure_m2,
            &summary.certified_current_area_enclosure_m2))
      return Fail(S::Unrepresentable, forecast,
          "Certified current parent-area summary overflowed", p);
    if (!summary.certified_parents ||
        result.minimum_scaled_jacobian_quality <
            summary.minimum_scaled_jacobian_quality) {
      summary.minimum_scaled_jacobian_quality =
          result.minimum_scaled_jacobian_quality;
      summary.minimum_quality_parent = p;
      summary.minimum_quality_local_facet =
          result.minimum_quality_local_facet;
    }
    if (result.approximation.total_error_upper_m >
        summary.maximum_approximation_upper_m) {
      summary.maximum_approximation_upper_m =
          result.approximation.total_error_upper_m;
      summary.maximum_approximation_parent = p;
    }
    ++summary.certified_parents;
  }
  if (summary.certified_parents + summary.skipped_parents !=
          summary.parents ||
      summary.active_parents + summary.removing_parents !=
          summary.certified_parents)
    return Fail(S::IdentityMismatch, forecast,
        "Current regularity summary is incomplete");
  *summary_output = summary;
  return {S::Ok, SIZE_MAX, SIZE_MAX,
          forecast.parents, forecast.facets, "OK"};
}

}  // namespace tlfea::contact::current_regularity
