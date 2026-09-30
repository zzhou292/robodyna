// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
extern "C" void rd_coated_coefficient(const double*, const int*, double*);
namespace coated_coefficient_test {
n::NativeCoatedMainCoefficientResult Oracle(const n::NativeCoatedMainCoefficientInput& in) {
  const auto& s=in.shell;const auto& v=in.solid;
  const double values[]{s.scale,s.element_thickness,s.property_thickness,s.young,
      v.fill,v.area,v.volume,v.bulk,v.controlled_bulk};
  const int flags[]{s.property_type,s.input_thickness_mode,
      s.layout==n::ShellLayout::Triangle3?3:4,v.incompressibility_control};
  double out[3];rd_coated_coefficient(values,flags,out);
  return {out[0],out[1],out[2]};
}
}
