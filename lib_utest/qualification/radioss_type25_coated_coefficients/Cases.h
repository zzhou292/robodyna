// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NativeOracle.h"
#include "../radioss_type25_coefficients/Assertions.h"
#include <vector>
namespace coated_coefficient_test {
inline n::NativeCoatedMainCoefficientInput Base() {
  n::NativeCoatedMainCoefficientInput in;
  in.shell.face=n::MainFaceKind::Coating;in.shell.layout=n::ShellLayout::Quad4;
  in.shell.property_type=1;in.shell.element_thickness=.25;in.shell.property_thickness=.5;
  in.shell.young=210000;
  in.solid.face=n::MainFaceKind::OrdinaryExterior;in.solid.layout=n::SolidLayout::EightSlot;
  in.solid.area=4;in.solid.volume=8;in.solid.bulk=87500;in.solid.controlled_bulk=175000;
  return in;
}
inline std::vector<n::NativeCoatedMainCoefficientInput> Cases() {
  std::vector<n::NativeCoatedMainCoefficientInput> out;
  for(auto layout:{n::ShellLayout::Triangle3,n::ShellLayout::Quad4})
    for(double bulk:{0.,26250.,87500.})for(double scale:{.125,1.,3.})for(int mode:{0,1}) {
      auto p=Base();p.shell.layout=layout;p.solid.bulk=bulk;
      p.shell.scale=p.solid.scale=scale;p.shell.input_thickness_mode=mode;out.push_back(p);
    }
  for(int control:{0,1,2}) {auto p=Base();p.solid.incompressibility_control=control;out.push_back(p);}
  for(double young:{0.,-0.})for(double bulk:{0.,-0.}) {
    auto p=Base();p.shell.young=young;p.solid.bulk=bulk;out.push_back(p);
  }
  for(double scale:{0.,-0.}) {auto p=Base();p.shell.scale=p.solid.scale=scale;out.push_back(p);}
  for(int property:{17,51}) {auto p=Base();p.shell.property_type=property;
    p.shell.element_thickness=0;out.push_back(p);}
  for(auto layout:{n::ShellLayout::Triangle3,n::ShellLayout::Quad4})
    for(double volume:{-8.,-0.125,-115.12871884667838}) {
      auto p=Base();p.shell.layout=layout;p.solid.volume=volume;out.push_back(p);
    }
  return out;
}
inline void Same(const n::NativeCoatedMainCoefficientResult& a,
    const n::NativeCoatedMainCoefficientResult& b,bool exact=false) {
  using type25_coefficient_test::Number;
  Number(a.primary_stiffness,b.primary_stiffness,exact||b.primary_stiffness==0);
  Number(a.partner_stiffness,b.partner_stiffness,exact||b.partner_stiffness==0);
  Number(a.solid_characteristic_length,b.solid_characteristic_length,exact);
}
struct Packet {n::NativeCoatedMainCoefficientInput input;};
struct Result {n::CoefficientStatus status; n::NativeCoatedMainCoefficientResult values;};
}
