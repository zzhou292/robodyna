// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NativeOracle.h"
#include "lib_src/collision/RadiossType25Coefficients.h"
#include <vector>
#include <limits>
#include <cmath>
namespace type25_coefficient_test {
inline n::NativeShellMainCoefficientInput Shell() {
  n::NativeShellMainCoefficientInput in;
  in.face = n::MainFaceKind::OrdinaryExterior; in.layout = n::ShellLayout::Quad4;
  in.property_type = 1; in.element_thickness = .25; in.property_thickness = .5;
  in.young = 210000; return in;
}
inline n::NativeSolidMainCoefficientInput Solid() {
  n::NativeSolidMainCoefficientInput in;
  in.face = n::MainFaceKind::OrdinaryExterior; in.layout = n::SolidLayout::EightSlot;
  in.fill = .75; in.area = 2.5; in.volume = 7.25;
  in.bulk = 175000; in.controlled_bulk = 350000; return in;
}
enum class Kind { Shell, Solid, Nodal, Secondary, Pair };
struct Packet {
  Kind kind = Kind::Shell;
  n::NativeShellMainCoefficientInput shell;
  n::NativeSolidMainCoefficientInput solid;
  n::NativeAccumulatedNodalCoefficients nodal;
  n::NativeSecondaryCoefficientInput secondary;
  n::NativePairCoefficientInput pair;
};
struct Result {
  n::CoefficientStatus status = n::CoefficientStatus::InvalidInput;
  double first = 0, second = 0;
};
TL_MATH_HOST_DEVICE inline Result Evaluate(const Packet& p) {
  Result result;
  if (p.kind == Kind::Shell) {
    n::NativeScalarCoefficient out;
    result.status = n::EvaluateNativeShellMainCoefficient(p.shell, &out); result.first = out.value;
  } else if (p.kind == Kind::Solid) {
    n::NativeSolidMainCoefficientResult out;
    result.status = n::EvaluateNativeSolidMainCoefficient(p.solid, &out);
    result.first = out.stiffness; result.second = out.characteristic_length;
  } else if (p.kind == Kind::Nodal) {
    n::NativeNodalCoefficientResult out;
    result.status = n::FinalizeNativeNodalCoefficient(p.nodal, &out);
    result.first = out.normalized_bulk; result.second = out.stiffness;
  } else if (p.kind == Kind::Secondary) {
    n::NativeScalarCoefficient out;
    result.status = n::EvaluateNativeSecondaryCoefficient(p.secondary, &out); result.first = out.value;
  } else {
    n::NativeScalarCoefficient out;
    result.status = n::EvaluateNativePairCoefficient({4, 0}, p.pair, &out); result.first = out.value;
  }
  return result;
}
inline Result Reference(const Packet& p) {
  Result result; result.status = n::CoefficientStatus::Ok;
  if (p.kind == Kind::Shell) result.first = Oracle(p.shell).value;
  else if (p.kind == Kind::Solid) {
    const auto out = Oracle(p.solid); result.first = out.stiffness; result.second = out.characteristic_length;
  } else if (p.kind == Kind::Nodal) {
    const auto out = Oracle(p.nodal); result.first = out.normalized_bulk; result.second = out.stiffness;
  } else if (p.kind == Kind::Secondary) result.first = Oracle(p.secondary).value;
  else result.first = Oracle(p.pair).value;
  return result;
}
inline std::vector<Packet> Cases() {
  std::vector<Packet> rows;
  for (auto layout : {n::ShellLayout::Quad4, n::ShellLayout::Triangle3})
    for (int property : {1, 11, 17, 51})
      for (int mode : {0, 1}) for (double thickness : {0., .25}) {
        Packet p; p.shell = Shell(); p.shell.layout = layout; p.shell.property_type = property;
        p.shell.input_thickness_mode = mode; p.shell.element_thickness = thickness; rows.push_back(p);
      }
  for (auto layout : {n::SolidLayout::EightSlot, n::SolidLayout::TenNode,
                     n::SolidLayout::TwentyNode, n::SolidLayout::SixteenNode})
    for (int control : {0, 1, 2}) for (double fill : {0., .75, 1.}) {
      Packet p; p.kind = Kind::Solid; p.solid = Solid(); p.solid.layout = layout;
      p.solid.incompressibility_control = control; p.solid.fill = fill; rows.push_back(p);
    }
  const double floor = 1. / (1e20 * 1e10);
  for (double volume : {0., floor/2, floor, std::nextafter(floor, 1.), .125, 4., 13.})
    for (int shells : {0, 3}) {
      Packet p; p.kind = Kind::Nodal;
      p.nodal = {volume, 2. * volume, 21., shells, 11.}; rows.push_back(p);
    }
  for (double existing : {0., -0., -2., 2.}) for (double scale : {-1., 0., .5, 1., 2.}) {
    Packet p; p.kind = Kind::Secondary; p.secondary = {existing, 17., scale}; rows.push_back(p);
  }
  for (auto input : {n::NativePairCoefficientInput{2800, 2410, 0, 1e30},
       n::NativePairCoefficientInput{2800, -2410, 0, 1e30},
       n::NativePairCoefficientInput{164000, 164000, 0, 1e30},
       n::NativePairCoefficientInput{5, 3, 4, 10},
       n::NativePairCoefficientInput{20, 30, 1, 10},
       n::NativePairCoefficientInput{-20, 30, 0, 10},
       n::NativePairCoefficientInput{2, 0, 0, 10},
       n::NativePairCoefficientInput{0, -0., 0, 10}}) {
    Packet p; p.kind = Kind::Pair; p.pair = input; rows.push_back(p);
  }
  return rows;
}
} // namespace type25_coefficient_test
