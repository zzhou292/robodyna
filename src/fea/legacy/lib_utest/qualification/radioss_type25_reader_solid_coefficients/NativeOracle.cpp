// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
extern "C" void rd_reader_coefficient(const double*,const int*,const double*,double*);
namespace reader_solid_test {
NativeResult Oracle(const Case& c) {
  const auto& a=c.input.first;
  const double v[]{a.scale,a.fill,a.area,a.volume,a.bulk,a.controlled_bulk,
      c.input.second_fill,c.input.second_bulk,c.unused_second_controlled_bulk};
  const int f[]{a.incompressibility_control,c.internal?1:0};
  double r[3];rd_reader_coefficient(v,f,c.second_coordinates.data(),r);
  return {{r[0],r[1]},r[2]};
}
}
