// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TestSupport.h"
#include <cstring>

namespace solid24_test {
template <std::size_t N>
inline bool SameValueBits(const std::array<double,N>& a,const std::array<double,N>& b) {
  return std::memcmp(a.data(),b.data(),sizeof(double)*N)==0;
}
inline s::ReferenceInput TotalReference(s::ReferenceInput input,
                                      s::WorkingLengthUnit units=s::WorkingLengthUnit::Metre) {
  input.profile.reference_strain=s::ReferenceStrain::TotalLagrangian10;
  input.profile.working_length=units;
  return input;
}
inline std::array<double,10> JacobianValues(const s::Reference& reference) {
  std::array<double,10> values{};
  if (const auto* jac=reference.reference_jacobian()) {
    std::copy(std::begin(jac->inverse),std::end(jac->inverse),values.begin());
    values[9]=jac->volume_m3;
  }
  return values;
}
inline bool JacobianAgree(const std::array<double,10>& a,const std::array<double,10>& b,
                          double roundoff_units=64) {
  double scale=0;
  for (unsigned n=0;n<9;++n) scale=std::max({scale,std::abs(a[n]),std::abs(b[n])});
  for (unsigned n=0;n<10;++n) {
    if (!std::isfinite(a[n]) || !std::isfinite(b[n])) return false;
    const double group=n<9?scale:std::abs(b[9]);
    if (std::abs(a[n]-b[n])>2e-11*std::abs(b[n])+
        roundoff_units*std::numeric_limits<double>::epsilon()*group) return false;
  }
  return true;
}
inline s::ReferenceInput FloorControl(s::WorkingLengthUnit units,double factor) {
  auto input=TotalReference(Brick(),units);
  const double scale=units==s::WorkingLengthUnit::Metre?1.0:.001;
  const double x=1e-7*scale,y=1e-7*scale,z=factor*1e-6*scale;
  const s::Vec3 nodes[8]{{0,0,0},{x,0,0},{x,y,0},{0,y,0},
                        {0,0,z},{x,0,z},{x,y,z},{0,y,z}};
  for (unsigned n=0;n<8;++n) input.position_m[n]=nodes[n];
  return input;
}
}  // namespace solid24_test
