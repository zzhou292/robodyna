// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/solid18/Solid18Reference.h"
#include <array>
#include <algorithm>
#include <cmath>
#include <cassert>
#include <cstring>

namespace solid18_test {
namespace s = tl::fea::solid18;
inline s::ReferenceInput Cube() {
  s::ReferenceInput in;
  in.source_element_id = 100;
  in.source_part_id = 200;
  in.source_section_id = 201;
  in.source_material_id = 202;
  in.density_kg_m3 = 1070;
  const s::Vec3 x[8] = {{0,0,0},{1,0,0},{1,1,0},{0,1,0},
                        {0,0,1},{1,0,1},{1,1,1},{0,1,1}};
  for (unsigned n = 0; n < 8; ++n) {
    in.source_node_id[n] = 1000+n;
    in.position_m[n] = x[n];
  }
  return in;
}
inline s::ReferenceInput Distorted() {
  auto in = Cube();
  in.position_m[6] = {1.3,.91,1.2};
  in.position_m[7] = {-.04,1.08,.85};
  return in;
}
template<class T> inline auto Bytes(const T& value) {
  std::array<unsigned char,sizeof(T)> result;
  std::memcpy(result.data(), &value, sizeof(value));
  return result;
}
inline bool Close(double a, double b, double tolerance=2e-11) {
  return std::isfinite(a) && std::isfinite(b) &&
         std::abs(a-b) <= tolerance*std::max({1e-18,std::abs(a),std::abs(b)});
}
// Named active values only. Object padding is not a comparison/serialization ABI.
inline constexpr unsigned ValueCount = 148;
inline auto Values(const s::Reference& r) {
  std::array<double,ValueCount> v{};
  std::size_t i = 0;
  const auto put = [&](const s::Vec3& x) {
    v[i++] = x.x;
    v[i++] = x.y;
    v[i++] = x.z;
  };
  const auto& g = r.geometry();
  for (double x : g.frame.v) v[i++] = x;
  for (const auto& x : g.native_position_m) put(x);
  for (double x : g.center_scaled_jacobian_m.v) v[i++] = x;
  for (const auto& x : g.higher_mode_m) put(x);
  for (const auto& p : g.point) {
    for (double x : p.scaled_jacobian_m.v) v[i++] = x;
    v[i++] = p.initial_volume_m3;
  }
  v[i++] = g.center_volume_m3;
  v[i++] = g.integrated_volume_m3;
  v[i++] = g.inverse_center_face_scale_per_m2;
  v[i++] = g.characteristic_length_m;
  for (double x : r.mass().source_nodal_mass_kg) v[i++] = x;
  v[i++] = r.mass().element_mass_kg;
  v[i++] = r.mass().initial_global_density_kg_m3;
  assert(i == v.size());
  return v;
}
}  // namespace solid18_test
