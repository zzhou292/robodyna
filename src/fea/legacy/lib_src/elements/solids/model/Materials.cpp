// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
#include "../../../materials/law36/Prepare.h"
#include "../../../materials/law42/Prepare.h"
#include <algorithm>

namespace tl::fea::solids::model_detail {
std::uint64_t MaterialId(ModelInput input, std::size_t index) noexcept {
  return Visit(input, index, [](const auto& parent, std::size_t) {
    return parent.reference.input().source_material_id;
  });
}
ModelReport PlanMaterials(ModelInput input, ModelLimits limits, Scratch& scratch, Layout& layout) {
  const auto count = Count(input);
  scratch.materials.Prepare(count, [&](std::size_t i) { return MaterialId(input, i); });
  for (std::size_t i = 0; i < count; ++i) {
    if (!MaterialId(input, i)) return Error(ModelStatus::InvalidInput, "Source MID must be positive", input, i);
    const auto first = scratch.materials.First(MaterialId(input, i));
    if (LawAt(input, first) != LawAt(input, i))
      return Error(ModelStatus::MaterialMismatch, "One source MID declares different material laws", input, i);
    const auto report = Visit(input, i, [&](const auto& parent, std::size_t) -> ModelReport {
      const auto curve = Curve(parent.material);
      if (RequiresCurve(parent.material) &&
          (curve.count < 2 || curve.count > 1024 || !Range(curve.x, curve.count) || !Range(curve.y, curve.count)))
        return Error(ModelStatus::InvalidInput, "Material curve range is invalid", input, i);
      if (!RequiresCurve(parent.material) && (curve.count || curve.x || curve.y))
        return Error(ModelStatus::InvalidInput, "Curve-free material must have an empty curve", input, i);
      if (first == i) {
        if (curve.count > limits.max_curve_points-layout.curve_points)
          return Error(ModelStatus::ResourceLimit, "Owned material curve pool exceeds cap", input, i);
        layout.curve_points += curve.count;
        scratch.material_indices[i] = MaterialCount(layout, parent.material)++;
      } else scratch.material_indices[i] = scratch.material_indices[first];
      return {};
    });
    if (!report) return report;
    if (layout.count36+layout.count42+layout.count44+layout.count90 > limits.max_materials)
      return Error(ModelStatus::ResourceLimit, "Unique source material count exceeds cap", input, i);
  }
  return {};
}
CurveSpan CopyCurve(CurveSpan curve, std::size_t& cursor, Storage& out) {
  if (!curve.count) return {};
  auto* x = out.curves+cursor;
  auto* y = x+curve.count;
  cursor += 2*curve.count;
  std::copy_n(curve.x, curve.count, x);
  std::copy_n(curve.y, curve.count, y);
  return {x, y, curve.count};
}
bool SameCurves(CurveSpan a, CurveSpan b) noexcept {
  if (a.count != b.count) return false;
  if (!a.count) return !a.x && !a.y && !b.x && !b.y;
  for (std::size_t i = 0; i < a.count; ++i)
    if (!Same(a.x[i], b.x[i]) || !Same(a.y[i], b.y[i])) return false;
  return true;
}
ModelReport CopyMaterial(const solid18::Material& original, std::uint64_t mid,
    bool first, std::size_t slot, std::size_t& cursor, Storage& out) {
  auto& record = out.material36[slot];
  if (first) {
    record.source_material_id = mid;
    const auto curve = CopyCurve(Curve(original), cursor, out);
    if (tl::material::law36::Prepare(original.young_pa, original.poisson_ratio,
        original.density_kg_m3, {curve.x, curve.y, curve.count}, record.value) != tl::material::law36::Status::Ok ||
        !Same(record.value, original))
      return {ModelStatus::InvalidInput, "Prepared LAW36 material or curve values differ"};
  } else if (!Same(record.value, original))
    return {ModelStatus::MaterialMismatch, "Repeated source MID has different LAW36 values"};
  return {};
}
ModelReport CopyMaterial(const solid24::Material& original, std::uint64_t mid,
    bool first, std::size_t slot, std::size_t&, Storage& out) {
  auto& record = out.material42[slot];
  if (first) {
    record.source_material_id = mid;
    if (tl::material::law42::Prepare(original.mu_pa, original.poisson_ratio,
        original.density_kg_m3, original.tension_cutoff_pa, record.value) != tl::material::law42::Status::Ok ||
        !Same(record.value, original))
      return {ModelStatus::InvalidInput, "Prepared LAW42 material values differ"};
  } else if (!Same(record.value, original))
    return {ModelStatus::MaterialMismatch, "Repeated source MID has different LAW42 values"};
  return {};
}
ModelReport CopyMaterials(ModelInput input, const Scratch& scratch, const Layout& layout, Storage& out) {
  std::size_t cursor = 0;
  for (std::size_t i = 0; i < Count(input); ++i) {
    const auto mid = MaterialId(input, i);
    const auto first = scratch.materials.First(mid);
    const auto report = Visit(input, i, [&](const auto& parent, std::size_t) {
      return CopyMaterial(parent.material, mid, first == i, scratch.material_indices[i], cursor, out);
    });
    if (!report) return Error(report.status, report.message, input, i);
  }
  if (cursor != 2*layout.curve_points)
    return Error(ModelStatus::InvalidInput, "Owned curve pool extent differs", input);
  return {};
}
} // namespace tl::fea::solids::model_detail
