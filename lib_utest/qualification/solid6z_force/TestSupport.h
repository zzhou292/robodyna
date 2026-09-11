// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/solid6z/Solid6zForce.h"
#include "lib_src/elements/solid6z/Solid6zReference.h"
#include <cmath>
#include <stdexcept>

namespace solid6z_force_test {
namespace s = tl::fea::solid6z;
inline s::ReferenceInput ReferenceInput() {
  s::ReferenceInput input;
  input.source_element_id = 71;
  input.source_part_id = 72;
  input.source_section_id = 73;
  input.source_material_id = 74;
  input.density_kg_m3 = 1980;
  const s::Vec3 x[6]{{0,0,0},{.024,0,0},{0,.021,0},
                     {.002,-.001,.013},{.025,.001,.014},{-.001,.022,.012}};
  for (unsigned n = 0; n < 6; ++n) {
    input.source_node_id[n] = 200+n;
    input.position_m[n] = x[n];
  }
  return input;
}
inline s::Reference Reference(s::ReferenceInput input = ReferenceInput()) {
  s::Reference result;
  if (s::InitializeReference(input,result) != s::Status::Success) throw std::runtime_error("wedge reference");
  return result;
}
inline s::Material Material() {
  s::Material result;
  if (tl::material::law42::Prepare(24e6,.463,1980,1e26,result) != tl::material::law42::Status::Ok)
    throw std::runtime_error("LAW42 material");
  return result;
}
inline s::History Initial(const s::Reference& reference, const s::Material& material,
                          s::ForceProfile profile = {}) {
  s::History history;
  if (s::InitializeHistory(reference,material,profile,history) != s::Status::Success)
    throw std::runtime_error("wedge history");
  return history;
}
inline s::Vec3 Position(s::Vec3 x, unsigned node, double phase) {
  const double stretch = std::sin(phase);
  const double spin = .6*(1-std::cos(phase*.5));
  const double c = std::cos(spin), sn = std::sin(spin);
  const double pattern[6]{1,-.7,.3,-.8,.6,-.4};
  const double n = pattern[node];
  const double a = (1+.18*stretch)*x.x+.06*stretch*x.y+.0003*n*std::sin(2*phase);
  const double b = (1-.09*stretch)*x.y+.07*stretch*x.z+.0002*n*std::sin(3*phase);
  const double z = (1+.11*stretch)*x.z+.05*stretch*x.x+.0004*n*std::sin(phase);
  return {c*a-sn*b,sn*a+c*b,z};
}
inline s::PrescribedInterval Path(const s::Reference& reference, unsigned step) {
  constexpr double h = 1e-6;
  constexpr double increment = 6.2831853071795864769/400;
  s::PrescribedInterval interval;
  interval.base_time_s = step*h;
  interval.dt_s = h;
  interval.sample_index = step;
  for (unsigned n = 0; n < 6; ++n) {
    const auto x = reference.input().position_m[n];
    const auto a = Position(x,n,increment*step);
    const auto b = Position(x,n,increment*(step+1));
    interval.position_endpoint_m[n] = b;
    interval.velocity_midpoint_m_s[n] = {(b.x-a.x)/h,(b.y-a.y)/h,(b.z-a.z)/h};
  }
  return interval;
}
} // namespace solid6z_force_test
