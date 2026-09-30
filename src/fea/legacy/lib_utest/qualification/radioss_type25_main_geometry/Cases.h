// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NativeOracle.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>
namespace main_geometry_test {
inline n::NativeExteriorMainGeometryInput Basic(bool triangle=false) {
  n::NativeExteriorMainGeometryInput in;
  in.layout=triangle?n::ShellLayout::Triangle3:n::ShellLayout::Quad4;
  const n::Vector hex[]{{-1,-1,0},{1,-1,0},{1,1,0},{-1,1,0},
                         {-1,-1,2},{1,-1,2},{1,1,2},{-1,1,2}};
  std::copy_n(hex,8,in.solid_raw);
  std::copy_n(hex,4,in.face);
  if(triangle) {
    // Literal native PENTA6 reader packet, with raw fourth/eighth repetitions.
    const n::Vector wedge[]{{-1,-1,0},{1,-1,0},{0,1,0},{-1,-1,0},
                            {-1,-1,2},{1,-1,2},{0,1,2},{-1,-1,2}};
    std::copy_n(wedge,8,in.solid_raw);
    in.face[0]=wedge[0];in.face[1]=wedge[1];in.face[2]=in.face[3]=wedge[2];
  }
  return in;
}
inline void Transform(n::NativeExteriorMainGeometryInput& in,double scale,n::Vector shift,unsigned rotate) {
  const auto point=[&](n::Vector& x) {
    x={x.x*scale+shift.x,x.y*scale+shift.y,x.z*scale+shift.z};
    for(unsigned i=0;i<rotate;++i)x={x.z,x.x,x.y};
  };
  for(auto& x:in.face)point(x);
  for(auto& x:in.solid_raw)point(x);
}
inline void Reverse(n::NativeExteriorMainGeometryInput& in) {
  if(in.layout==n::ShellLayout::Triangle3)std::swap(in.face[0],in.face[1]);
  else std::reverse(std::begin(in.face),std::end(in.face));
}
inline std::vector<n::NativeExteriorMainGeometryInput> Cases() {
  std::vector<n::NativeExteriorMainGeometryInput> out;
  for(bool triangle:{false,true})for(double scale:{1.,.125,8.,1.e-11,1.e-80})
    for(unsigned rotation=0;rotation<3;++rotation)for(bool reversed:{false,true}) {
      auto in=Basic(triangle);
      Transform(in,scale,{},rotation);
      if(reversed)Reverse(in);
      out.push_back(in);
    }
  // Deterministic non-dyadic warped bricks: all face coordinates are real raw
  // solid slots. Native expected values are computed independently at runtime.
  std::uint32_t state=0x98139u;
  for(unsigned row=0;row<96;++row) {
    auto in=Basic();
    for(auto& x:in.solid_raw) {
      double* fields[]{&x.x,&x.y,&x.z};
      for(auto field:fields) {
        state=1664525u*state+1013904223u;
        *field += double(int((state>>8)%201)-100)/1009.;
      }
    }
    std::copy_n(in.solid_raw,4,in.face);
    Transform(in,1.,{123.125,-87.25,11.5},row%3);
    if(row%2)Reverse(in);
    out.push_back(in);
  }
  for(bool triangle:{false,true})for(unsigned mask=0;mask<8;++mask) {
    auto in=Basic(triangle);
    for(auto& x:in.face) x.z=mask&1?-0.:0.;
    for(auto& x:in.solid_raw) {
      if(mask&2)x.x=-x.x;
      x.z=mask&4?-0.:0.;
    }
    out.push_back(in);
  }
  for(bool triangle:{false,true})for(double projection:{-1.,-std::numeric_limits<double>::denorm_min(),
        -0.,0.,std::numeric_limits<double>::denorm_min(),1.}) {
    auto in=Basic(triangle);
    for(auto& x:in.solid_raw)x.z=projection;
    out.push_back(in);
  }
  return out;
}
inline n::NativeExteriorMainGeometryResult Sentinel() {
  n::NativeExteriorMainGeometryResult out;
  out.normal_before_orientation={71.,-13.,-0.};
  out.area=91.;out.signed_volume=-81.;out.center_projection=19.;
  for(unsigned i=0;i<4;++i)out.source_corner[i]=17+i;
  out.reversed=true;
  return out;
}
inline std::vector<n::NativeExteriorMainGeometryInput> Invalid() {
  std::vector<n::NativeExteriorMainGeometryInput> out(6,Basic());
  out[0].layout=n::ShellLayout::Unspecified;
  out[1].face[0].x=std::numeric_limits<double>::quiet_NaN();
  out[2]=Basic(true);out[2].face[3].z=-0.; // Same numeric point, wrong repeated identity bits.
  out[3].solid_raw[0].z=std::numeric_limits<double>::infinity();
  out[4].face[0].x=-std::numeric_limits<double>::max();
  out[4].face[2].x=std::numeric_limits<double>::max();
  for(auto& x:out[5].solid_raw)x.x=std::numeric_limits<double>::max();
  return out;
}
inline n::CoefficientStatus InvalidStatus(unsigned i) {
  return i==0?n::CoefficientStatus::UnsupportedProfile:
    i>=4?n::CoefficientStatus::NonfiniteResult:n::CoefficientStatus::InvalidInput;
}
} // namespace main_geometry_test
