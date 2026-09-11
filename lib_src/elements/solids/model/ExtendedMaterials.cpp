// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
#include "../../../materials/law44/solid/Prepare.h"
#include "../../../materials/law90/Prepare.h"

namespace tl::fea::solids::model_detail {
namespace {
tl::material::law90::PreparationInput CanonicalInput(const tl::material::law90::ReaderValues& r) {
  tl::material::law90::PreparationInput input;
  input.density_kg_m3 = r.density_kg_m3;
  input.reference_density_kg_m3 = r.reference_density_kg_m3;
  input.card_young_pa = r.card_young_pa;
  input.poisson_ratio = r.poisson_ratio;
  input.contact_modulus_pa = r.contact_modulus_pa;
  input.tension_cutoff_pa = r.tension_cutoff_pa;
  input.hysteresis = r.hysteresis;
  input.shape = r.shape;
  input.alpha = r.alpha;
  input.curve_scale = r.curve_scale;
  input.curve_rate_s_inverse = r.curve_rate_s_inverse;
  input.filter_cutoff_hz = r.filter_cutoff_hz;
  input.smooth = r.smooth;
  input.tension_flag = r.tension_flag;
  input.failure_mode = r.failure_mode;
  return input;
}
}
ModelReport CopyMaterial(const solid18::law44::Material& original, std::uint64_t mid,
    bool first, std::size_t slot, std::size_t& cursor, Storage& out) {
  auto& record = out.material44[slot];
  if (first) {
    record.source_material_id = mid;
    const auto curve = CopyCurve(Curve(original), cursor, out);
    if (tl::material::law44::solid::Prepare(original.material,
        {curve.x, curve.y, static_cast<std::uint32_t>(curve.count)}, record.value) != tl::material::law44::solid::Status::Ok ||
        !Same(record.value, original))
      return {ModelStatus::InvalidInput, "Prepared LAW44 material or curve values differ"};
  } else if (!Same(record.value, original))
    return {ModelStatus::MaterialMismatch, "Repeated source MID has different LAW44 values"};
  return {};
}
ModelReport CopyMaterial(const tl::material::law90::PreparedMaterial& original, std::uint64_t mid,
    bool first, std::size_t slot, std::size_t& cursor, Storage& out) {
  auto& record = out.material90[slot];
  if (first) {
    if (!original.initialized()) return {ModelStatus::InvalidInput, "LAW90 material is not prepared"};
    record.source_material_id = mid;
    const auto curve = CopyCurve(Curve(original), cursor, out);
    // Reconstruct the already resolved reader state, not raw source defaults.
    // Full named reader/updated equality below authenticates the rebind.
    if (tl::material::law90::PrepareSI(CanonicalInput(original.reader()),
        {curve.x, curve.y, static_cast<std::uint32_t>(curve.count)}, record.value) != tl::material::law90::Status::Ok ||
        !Same(record.value, original))
      return {ModelStatus::InvalidInput, "Prepared LAW90 material or curve values differ"};
  } else if (!Same(record.value, original))
    return {ModelStatus::MaterialMismatch, "Repeated source MID has different LAW90 values"};
  return {};
}
} // namespace tl::fea::solids::model_detail
