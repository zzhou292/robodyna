// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include <stdexcept>
namespace type25_coefficient_test {
extern "C" void rd_type25_shell_coefficient(const double*, const int*, double*);
extern "C" void rd_type25_solid_coefficient(const double*, const int*, double*);
extern "C" void rd_type25_nodal_coefficient(const double*, const int*, double*);
extern "C" void rd_type25_secondary_coefficient(const double*, double*);
extern "C" void rd_type25_pair_coefficient(const double*, double*);
n::NativeScalarCoefficient Oracle(const n::NativeShellMainCoefficientInput& in) {
  if (in.face != n::MainFaceKind::OrdinaryExterior || in.scale < 0 ||
      in.property_type == 52 || (in.stack_material > 0 &&
          (in.property_type == 11 || in.property_type == 17 || in.property_type == 51)))
    throw std::invalid_argument("Unselected native shell reference branch");
  const double values[]{in.scale, in.element_thickness, in.property_thickness, in.young};
  const int flags[]{in.property_type, in.input_thickness_mode,
      in.layout == n::ShellLayout::Triangle3 ? 3 : 4};
  n::NativeScalarCoefficient out;
  rd_type25_shell_coefficient(values, flags, &out.value); return out;
}
n::NativeSolidMainCoefficientResult Oracle(const n::NativeSolidMainCoefficientInput& in) {
  if (in.face != n::MainFaceKind::OrdinaryExterior) throw std::invalid_argument("Unselected native solid face");
  int family = 0;
  switch (in.layout) {
    case n::SolidLayout::EightSlot: family = 1; break;
    case n::SolidLayout::TenNode: family = 2; break;
    case n::SolidLayout::TwentyNode: family = 3; break;
    case n::SolidLayout::SixteenNode: family = 4; break;
    default: throw std::invalid_argument("Unknown native solid family");
  }
  const double values[]{in.scale, in.fill, in.area, in.volume, in.bulk, in.controlled_bulk};
  const int flags[]{family, in.incompressibility_control}; double result[2];
  rd_type25_solid_coefficient(values, flags, result);
  return {result[0], result[1]};
}
n::NativeNodalCoefficientResult Oracle(const n::NativeAccumulatedNodalCoefficients& in) {
  const double values[]{in.volume, in.bulk_volume, in.young_thickness_sum, in.existing_stiffness};
  double result[2];
  rd_type25_nodal_coefficient(values, &in.shell_incidence_count, result);
  return {result[0], result[1]};
}
n::NativeScalarCoefficient Oracle(const n::NativeSecondaryCoefficientInput& in) {
  const double values[]{in.existing, in.global_stiffness, in.scale}; n::NativeScalarCoefficient out;
  rd_type25_secondary_coefficient(values, &out.value); return out;
}
n::NativeScalarCoefficient Oracle(const n::NativePairCoefficientInput& in) {
  const double values[]{in.main, in.secondary, in.minimum, in.maximum}; n::NativeScalarCoefficient out;
  rd_type25_pair_coefficient(values, &out.value); return out;
}
} // namespace type25_coefficient_test
