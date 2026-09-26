// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include <cmath>
#include <stdexcept>
extern "C" void rd_reader_geometry(const double*,const int*,const int*,double*,int*,int*);
extern "C" void rd_reader_raw_volume(const double*,double*);
namespace reader_main_geometry_test {
namespace {
void Point(n::Vector x,double* p) {
  if(!std::isfinite(x.x)||!std::isfinite(x.y)||!std::isfinite(x.z))
    throw std::invalid_argument("Nonfinite native reader geometry packet");
  p[0]=x.x;p[1]=x.y;p[2]=x.z;
}
}
Observation InternalOracle(const n::NativeExteriorMainGeometryInput& in) {
  if(in.layout!=n::ShellLayout::Quad4&&in.layout!=n::ShellLayout::Triangle3)
    throw std::invalid_argument("Native reader geometry layout differs");
  double packet[36];
  for(unsigned i=0;i<4;++i)Point(in.face[i],packet+3*i);
  for(unsigned i=0;i<8;++i)Point(in.solid_raw[i],packet+3*(i+4));
  const int layout=in.layout==n::ShellLayout::Triangle3?3:4,branch=2;
  if(layout==3&&(!tl::math::SameScalarBits(in.face[2].x,in.face[3].x)||
      !tl::math::SameScalarBits(in.face[2].y,in.face[3].y)||
      !tl::math::SameScalarBits(in.face[2].z,in.face[3].z)))
    throw std::invalid_argument("Native T3 repeated identity differs");
  double result[6];int slots[4],reversed;
  rd_reader_geometry(packet,&layout,&branch,result,slots,&reversed);
  Observation out;
  out.value={{result[0],result[1],result[2]},result[3],result[4]};
  // R(6)/DDS is undefined at the original IC>=2 return and is never read.
  for(unsigned k=0;k<4;++k) {
    if(slots[k]<0||slots[k]>3)throw std::runtime_error("Invalid native reader slot observation");
    out.source_corner[k]=unsigned(slots[k]);
  }
  if(reversed!=0)throw std::runtime_error("Internal native primary unexpectedly reversed");
  out.reversed=false;return out;
}
double VolumeOracle(const n::Vector (&raw)[8]) {
  double packet[24],value;
  for(unsigned i=0;i<8;++i)Point(raw[i],packet+3*i);
  rd_reader_raw_volume(packet,&value);return value;
}
}
