// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include <cmath>
#include <stdexcept>
extern "C" void rd_main_geometry(const double*,const int*,double*,int*,int*);
extern "C" double rd_main_em20();
namespace main_geometry_test {
n::NativeExteriorMainGeometryResult Oracle(const n::NativeExteriorMainGeometryInput& in) {
  if (in.layout!=n::ShellLayout::Quad4 && in.layout!=n::ShellLayout::Triangle3)
    throw std::invalid_argument("Native geometry fixture requires Q4/T3");
  double packet[36];
  for (unsigned i=0;i<12;++i) {
    const auto x=i<4?in.face[i]:in.solid_raw[i-4];
    if (!std::isfinite(x.x)||!std::isfinite(x.y)||!std::isfinite(x.z))
      throw std::invalid_argument("Nonfinite native geometry fixture");
    packet[3*i]=x.x;packet[3*i+1]=x.y;packet[3*i+2]=x.z;
  }
  const int layout=in.layout==n::ShellLayout::Triangle3?3:4;
  if (layout==3 && (!tl::math::SameScalarBits(in.face[2].x,in.face[3].x) ||
      !tl::math::SameScalarBits(in.face[2].y,in.face[3].y) ||
      !tl::math::SameScalarBits(in.face[2].z,in.face[3].z)))
    throw std::invalid_argument("Inconsistent native T3 repeated identity");
  double out[6];int slots[4],reversed;
  rd_main_geometry(packet,&layout,out,slots,&reversed);
  n::NativeExteriorMainGeometryResult result;
  result.normal_before_orientation={out[0],out[1],out[2]};
  result.area=out[3];result.signed_volume=out[4];result.center_projection=out[5];
  for(unsigned i=0;i<4;++i) {
    if(slots[i]<0||slots[i]>3) throw std::runtime_error("Invalid native permutation observation");
    result.source_corner[i]=unsigned(slots[i]);
  }
  if(reversed!=0&&reversed!=1) throw std::runtime_error("Invalid native reversal observation");
  result.reversed=reversed!=0;
  return result;
}
double NativeEm20() { return rd_main_em20(); }
} // namespace main_geometry_test
