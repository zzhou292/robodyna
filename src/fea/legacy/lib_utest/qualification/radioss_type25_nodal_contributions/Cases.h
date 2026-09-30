// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NativeOracle.h"
#include <cmath>
#include <stdexcept>
#include <limits>
#include <vector>
namespace type25_contribution_test {
inline n::NativeSpringNodalInput Spring(n::SpringNodalKind kind=n::SpringNodalKind::Type13) {
  return {kind,1,1,{{3.,2.},{5.,3.},{7.,4.}},2.};
}
inline std::vector<n::NativeSolidNodalInput> SolidCases() {
  std::vector<n::NativeSolidNodalInput> result;
  for(auto kind:{n::SolidNodalKind::Hex8,n::SolidNodalKind::Penta6}) {
    for(double volume:{0.,-0.,8.,-.5,1e-300})for(double fill:{0.,-0.,.5,1.,-1.})
      result.push_back({kind,volume,fill,3.});
    result.push_back({kind,1e-300,1e300,1e-300});
    result.push_back({kind,1.,1.,-0.});
  }
  return result;
}
inline std::vector<n::NativeSpringNodalInput> SpringCases() {
  std::vector<n::NativeSpringNodalInput> result;
  for(auto kind:{n::SpringNodalKind::Type13,n::SpringNodalKind::Type25}) {
    for(int mode:{-2,0,1,3}) {
      auto in=Spring(kind);in.length_mode=mode;
      if(mode<=0)in.geometric_length=std::numeric_limits<double>::quiet_NaN();
      if(kind==n::SpringNodalKind::Type25)in.translation[2]={std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN()};
      result.push_back(in);
    }
    for(unsigned signs=0;signs<8;++signs) {
      auto in=Spring(kind);
      for(unsigned i=0;i<3;++i)in.translation[i]={(signs&(1u<<i))?-0.:0.,1.};
      result.push_back(in);
    }
    auto negative=Spring(kind);negative.translation[0]={-9.,1.};negative.translation[1]={-5.,1.};negative.translation[2]={-7.,1.};result.push_back(negative);
    for(double length:{0.,std::nextafter(n::native_constant::em30,0.),n::native_constant::em30,
        std::nextafter(n::native_constant::em30,std::numeric_limits<double>::infinity())}) {
      auto in=Spring(kind);in.geometric_length=length;
      for(unsigned i=0;i<3;++i)in.translation[i]={double(i+1)*1e-300,1.};
      result.push_back(in);
    }
  }
  return result;
}
inline double ReferenceSpring(const n::NativeSpringNodalInput& in) {
  double length=1.;
  if(in.length_mode>0)length=in.geometric_length;
  else {
    std::array<double,6> poison;poison.fill(std::numeric_limits<double>::quiet_NaN());
    const auto prepared=OracleLength(in.kind,in.length_mode,poison);
    if(prepared.diagnostics||prepared.value!=1.)throw std::runtime_error("Native constant XL branch differs");
    length=prepared.value;
  }
  return OracleSpringPrepared(in,length);
}
} // namespace type25_contribution_test
