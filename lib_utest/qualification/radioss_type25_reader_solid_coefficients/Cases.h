// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NativeOracle.h"
#include "../radioss_type25_coefficients/Assertions.h"
#include <vector>
#include <algorithm>
#include <limits>
namespace reader_solid_test {
inline Case Base(bool internal=true) {
  Case c;c.internal=internal;
  c.input.first.face=internal?n::MainFaceKind::Internal:n::MainFaceKind::OrdinaryExterior;
  auto& a=c.input.first;a.layout=n::SolidLayout::EightSlot;a.scale=.75;a.fill=.375;a.area=2.75;a.volume=3;
  a.bulk=87500;a.controlled_bulk=175000;
  c.input.second_fill=.625;c.input.second_bulk=212500;c.input.second_volume=12;
  c.second_coordinates={0,0,0,2,0,0,2,3,0,0,3,0,0,0,2,2,0,2,2,3,2,0,3,2};
  return c;
}
inline std::vector<Case> Cases() {
  std::vector<Case> out;
  for(bool internal:{false,true})for(int control:{0,1,2})
    for(double first_volume:{3.,-3.})for(double z:{2.,-2.})for(double scale:{.125,1.}) {
      auto c=Base(internal);c.input.first.incompressibility_control=control;
      c.input.first.volume=first_volume;c.input.first.scale=scale;
      for(unsigned k=4;k<8;++k)c.second_coordinates[3*k+2]=z;
      c.input.second_volume=6*z;c.unused_second_controlled_bulk=9e9;
      out.push_back(c);
    }
  for(double zero:{0.,-0.})for(bool internal:{false,true}) {
    auto c=Base(internal);c.input.first.scale=zero;out.push_back(c);
    c=Base(internal);c.input.first.fill=zero;c.input.second_fill=zero;out.push_back(c);
  }
  return out;
}
inline n::CoefficientStatus Evaluate(const Case& c,n::NativeSolidMainCoefficientResult* r) {
  return c.internal?n::EvaluateNativeInternalSolidMainCoefficient(c.input,r):
      n::EvaluateNativeReaderSolidMainCoefficient(c.input.first,r);
}
inline void Same(const n::NativeSolidMainCoefficientResult& a,const n::NativeSolidMainCoefficientResult& b) {
  using type25_coefficient_test::Number;
  Number(a.stiffness,b.stiffness,b.stiffness==0);
  Number(a.characteristic_length,b.characteristic_length,b.characteristic_length==0);
}
struct Result {n::CoefficientStatus status;n::NativeSolidMainCoefficientResult value;};
}
