#pragma once
#include "OriginalCurves.h"
#include "lib_src/elements/sections/ShellLayeredJ2Work.h"
#include <array>
#include <cstring>
#include <stdexcept>

namespace continuation_test {
namespace mat=tl::material;
namespace sec=tl::fea::sections;
using Policy=mat::ShellPlasticityCurveContinuation;
using Status=mat::TabulatedShellPlasticityStatus;
using Parameters=mat::TabulatedShellPlasticityParameters;
using History=mat::TabulatedShellPlasticityHistory;
using Input=mat::TabulatedShellPlasticityInput;
using Result=mat::TabulatedShellPlasticityResult;
inline Parameters Prepare(OriginalCurve c, bool rate=false, Policy policy=Policy::NativeLastSegment) {
  Parameters p;
  const mat::TabulatedShellPlasticityRate r=rate?
      mat::TabulatedShellPlasticityRate{true,8000.,8.,10000.}:mat::TabulatedShellPlasticityRate{};
  if(mat::PrepareTabulatedShellPlasticity(200e9,.3,7890,{c.x,c.y,c.count},r,policy,p)!=Status::Ok)
    throw std::runtime_error("Continuation fixture material");
  return p;
}
inline Input Increment(const Parameters& p, double dx=.004) {
  Input in;
  in.transverse_shear_modulus=p.shear_modulus*(5./6.);
  in.strain_increment[0]=dx;
  in.strain_increment[1]=-.3*dx;
  in.dt=1.e-6;
  in.total_strain_rate_per_s=p.rate.enabled?1000.:0.;
  return in;
}
template<class T> auto Bytes(const T& value) {
  std::array<unsigned char,sizeof(T)> out{};
  std::memcpy(out.data(),&value,sizeof value);
  return out;
}
} // namespace continuation_test
