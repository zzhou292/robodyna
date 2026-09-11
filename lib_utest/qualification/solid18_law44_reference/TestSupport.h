#pragma once
#include "lib_src/elements/solid18/law44/Reference.h"
#include "lib_utest/qualification/solid18_reference/TestSupport.h"

namespace rear18_test {
namespace s = tl::fea::solid18;
namespace law = s::law44;
inline s::ReferenceInput Cube() {
  auto input = solid18_test::Cube();
  input.profile = law::Profile();
  input.density_kg_m3 = 7.8900e-9*1e12;
  return input;
}
inline s::ReferenceInput Collapsed() {
  auto input = Cube();
  input.source_node_id[5] = input.source_node_id[4];
  input.source_node_id[7] = input.source_node_id[6];
  input.position_m[4] = input.position_m[5] = {.5, 0, 1};
  input.position_m[6] = input.position_m[7] = {.5, 1, 1};
  return input;
}
inline std::array<double, solid18_test::ValueCount> Values(const law::Reference& reference) {
  // Same named observation layout as the existing independent native packet.
  std::array<double, solid18_test::ValueCount> result{};
  unsigned i = 0;
  const auto vector = [&](s::Vec3 v) { result[i++] = v.x; result[i++] = v.y; result[i++] = v.z; };
  const auto& g = reference.geometry();
  for (double v : g.frame.v) result[i++] = v;
  for (auto v : g.native_position_m) vector(v);
  for (double v : g.center_scaled_jacobian_m.v) result[i++] = v;
  for (auto v : g.higher_mode_m) vector(v);
  for (const auto& point : g.point) {
    for (double v : point.scaled_jacobian_m.v) result[i++] = v;
    result[i++] = point.initial_volume_m3;
  }
  result[i++] = g.center_volume_m3;
  result[i++] = g.integrated_volume_m3;
  result[i++] = g.inverse_center_face_scale_per_m2;
  result[i++] = g.characteristic_length_m;
  for (double v : reference.mass().source_nodal_mass_kg) result[i++] = v;
  result[i++] = reference.mass().element_mass_kg;
  result[i++] = reference.mass().initial_global_density_kg_m3;
  assert(i == result.size());
  return result;
}
}  // namespace rear18_test
