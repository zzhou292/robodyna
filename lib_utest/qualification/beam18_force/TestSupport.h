// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/beam18/Force.h"
#include "lib_utest/qualification/beam18_reference/TestSupport.h"
#include "lib_utest/qualification/solid_law44_point/source_fixture/OriginalCurve.h"
#include <array>
#include <cstring>
#include <limits>
#include <stdexcept>
namespace beam18_force_test {
namespace b = tl::fea::beam18;
inline b::Reference Reference() {
  auto input = beam18_test::Input();
  input.units = b::WorkingUnits::SI; input.young = 200e9; input.density = 7890; input.radius = .0045;
  input.position[0] = {0,0,0}; input.position[1] = {.1,0,0}; input.position[2] = {0,1,0};
  b::Reference result;
  if (b::InitializeReference(input,result) != b::Status::Success) throw std::runtime_error("reference");
  return result;
}
inline b::Material Material(const b::Reference& reference) {
  const bool working = reference.input().units == b::WorkingUnits::TonneMillimetreSecond;
  b::point::Material raw{reference.input().young*(working?1e6:1),reference.input().poisson,
      reference.input().density*(working?1e12:1),8000,8,10000,
      working ? tl::material::law44::solid::WorkingUnits::TonneMillimetreSecond :
          tl::material::law44::solid::WorkingUnits::SI};
  b::Material result;
  if (b::point::Prepare(raw,{law44_solid_test::X,law44_solid_test::Y,law44_solid_test::Count},result) != b::point::Status::Ok)
    throw std::runtime_error("material");
  return result;
}
inline b::PrescribedInterval Motion(const b::Reference& ref, const b::ForceHistory& history) {
  b::PrescribedInterval p;
  p.base_time_s = history.stamp().time_s; p.dt_s = 1e-6; p.sample_index = history.stamp().sample_index+1;
  for (unsigned n=0;n<2;++n) p.position_endpoint_m[n]=ref.geometry().endpoint_m[n];
  return p;
}
inline bool Near(double a,double b,double tolerance=1e-11) {
  return std::abs(a-b)<=tolerance*std::max({std::abs(a),std::abs(b),1e-20});
}
} // namespace beam18_force_test
