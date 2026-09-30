// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/beam18/Reference.h"
#include <array>
#include <cmath>
#include <cstring>
#include <gtest/gtest.h>

namespace beam18_test {
namespace beam = tl::fea::beam18;
inline beam::Input Input(double length = 16, double radius = 4.5) {
  beam::Input input{};
  input.source_element_id = 101;
  input.source_part_id = input.source_section_id = input.source_material_id = 2000514;
  input.source_node_id[0] = 11; input.source_node_id[1] = 22; input.source_node_id[2] = 33;
  input.position[0] = {100,200,300};
  input.position[1] = {100+length,200,300};
  input.position[2] = {101,230,300};
  input.units = beam::WorkingUnits::TonneMillimetreSecond;
  input.profile = beam::Profile::CircularFourPointStoredZero;
  input.radius = radius; input.density = 7.89e-9; input.young = 200000; input.poisson = .3;
  return input;
}
template<class T> auto Bytes(const T& input) {
  std::array<unsigned char,sizeof(T)> result{};
  std::memcpy(result.data(),&input,sizeof(T));
  return result;
}
inline std::array<double,32> Values(const beam::Reference& value) {
  std::array<double,32> result{};
  unsigned k = 0;
  for (auto p : value.section().point) { result[k++] = p.y; result[k++] = p.z; result[k++] = p.area; }
  const auto& s = value.section(); const auto& g = value.geometry(); const auto& n = value.native_mass();
  for (double v : {s.area,s.inertia_y,s.inertia_z,s.inertia_x,g.length,
      g.orientation_seed.x,g.orientation_seed.y,g.orientation_seed.z,
      n.endpoint_mass,n.endpoint_total_inertia,n.translation_stiffness,n.rotation_stiffness,
      n.interface_stiffness,s.membrane_damping,s.flexural_damping,
      value.endpoint().mass_kg,value.endpoint().native_total_inertia_kg_m2,
      value.endpoint().translation_stiffness_n_m,value.endpoint().rotation_stiffness_nm,
      value.endpoint().interface_stiffness_n_m}) result[k++] = v;
  return result;
}
} // namespace beam18_test
