// SPDX-License-Identifier: AGPL-3.0-or-later
#include "MaterialUpload.h"
#include "../../../materials/law90/Relocate.h"

namespace tl::fea::solids::batch_detail {
namespace {
std::size_t Offset44(const Model& model) noexcept {
  std::size_t offset = 0;
  for (const auto& m : model.materials36()) offset += 2 * m.value.curve.count;
  return offset;
}
std::size_t Offset90(const Model& model) noexcept {
  auto offset = Offset44(model);
  for (const auto& m : model.materials44()) offset += 2 * m.value.curve.count;
  return offset;
}
void CopyCurve(const double* x, const double* y, std::size_t count, double*& next) {
  if (!count) return;
  for (std::size_t p = 0; p < count; ++p) {
    next[p] = x[p];
    next[count + p] = y[p];
  }
  next += 2 * count;
}
} // namespace
solid18::Material ExpectedMaterial36(const Model& model, std::size_t index,
    double* curves) noexcept {
  std::size_t offset = 0;
  for (std::size_t m = 0; m < index; ++m) offset += 2 * model.materials36()[m].value.curve.count;
  auto result = model.materials36()[index].value;
  result.curve.plastic_strain = curves + offset;
  result.curve.yield_stress_pa = curves + offset + result.curve.count;
  return result;
}
solid18::law44::Material ExpectedMaterial44(const Model& model, std::size_t index,
    double* curves) noexcept {
  auto offset = Offset44(model);
  for (std::size_t m = 0; m < index; ++m) offset += 2 * model.materials44()[m].value.curve.count;
  auto result = model.materials44()[index].value;
  if (result.material.hardening == tl::material::law44::solid::HardeningKind::Analytic) return result;
  result.curve.plastic_strain = curves + offset;
  result.curve.yield_stress_pa = curves + offset + result.curve.count;
  return result;
}
BatchReport ExpectedMaterial90(const Model& model, std::size_t index,
    double* curves, solid18::total_strain::Material& output) noexcept {
  auto offset = Offset90(model);
  for (std::size_t m = 0; m < index; ++m) offset += 2 * model.materials90()[m].value.curve().count;
  const auto& source = model.materials90()[index].value;
  const auto count = source.curve().count;
  const auto status = tl::material::law90::RelocatePreparedCurve(source,
      {curves + offset, curves + offset + count, count}, output);
  if (status != tl::material::law90::Status::Ok)
    return {BatchStatus::InvalidInput, "Solid LAW90 immutable curve relocation rejected"};
  return {};
}
BatchReport UploadMaterials(const Model& model, util::HostArena& arena,
    const ArenaLayout& layout) {
  auto* curves = arena.Construct<double>(layout.curves);
  auto* m36 = arena.Construct<solid18::Material>(layout.material36);
  auto* m42 = arena.Construct<solid24::Material>(layout.material42);
  auto* m44 = arena.Construct<solid18::law44::Material>(layout.material44);
  auto* m90 = arena.Construct<solid18::total_strain::Material>(layout.material90);
  if ((layout.curves.count && !curves) || (layout.material36.count && !m36) ||
      (layout.material42.count && !m42) || (layout.material44.count && !m44) ||
      (layout.material90.count && !m90))
    return {BatchStatus::ResourceLimit, "Solid typed material upload layout differs"};
  auto* cursor = curves;
  for (std::size_t m = 0; m < model.materials36().size(); ++m) {
    const auto& c = model.materials36()[m].value.curve;
    CopyCurve(c.plastic_strain, c.yield_stress_pa, c.count, cursor);
    m36[m] = ExpectedMaterial36(model, m, curves);
  }
  for (std::size_t m = 0; m < model.materials42().size(); ++m) m42[m] = model.materials42()[m].value;
  for (std::size_t m = 0; m < model.materials44().size(); ++m) {
    const auto& c = model.materials44()[m].value.curve;
    CopyCurve(c.plastic_strain, c.yield_stress_pa, c.count, cursor);
    m44[m] = ExpectedMaterial44(model, m, curves);
  }
  for (std::size_t m = 0; m < model.materials90().size(); ++m) {
    const auto c = model.materials90()[m].value.curve();
    CopyCurve(c.compression_strain, c.stress_pa, c.count, cursor);
    const auto report = ExpectedMaterial90(model, m, curves, m90[m]);
    if (!report) return report;
  }
  return {};
}
BatchReport RebaseCurves(const Model& model, const ArenaLayout& layout, void* device,
    Storage& header) noexcept {
  auto* curves = util::ArenaPointer<double>(device, layout.curves);
  for (std::size_t m = 0; m < model.materials36().size(); ++m)
    header.material36[m] = ExpectedMaterial36(model, m, curves);
  for (std::size_t m = 0; m < model.materials44().size(); ++m)
    header.material44[m] = ExpectedMaterial44(model, m, curves);
  for (std::size_t m = 0; m < model.materials90().size(); ++m) {
    const auto report = ExpectedMaterial90(model, m, curves, header.material90[m]);
    if (!report) return report;
  }
  return {};
}
} // namespace tl::fea::solids::batch_detail
