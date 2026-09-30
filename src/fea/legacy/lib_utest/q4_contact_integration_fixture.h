#pragma once

#include "lib_src/collision/Q4ContactIntegration.h"

namespace q4_contact_test {
namespace sc = tlfea::contact;

// Distinct physical IDs and SoA/AoS storage exercise the actual C1 adapters.
struct Fixture {
  double position[12]{},velocity[12]{};
  double inverse[4]{.25,.5,0,1};
  std::uint8_t fixed[4]{6,6,7,6};
  sc::SurfaceQ4 parent{{3,0,2,1},73,42,2,0};
  TL_SURFACE_HD Fixture() {
    const double y[4]={.5,-.5,-.5,.5},z[4]={.5,.5,-.5,-.5};
    for (unsigned i=0;i<4;++i) {
      const auto n=parent.nodes[i];
      position[n+4]=y[i]; position[n+8]=z[i]; velocity[3*n]=i+.25;
    }
  }
  TL_SURFACE_HD void Gaps(double first,double second,double third,double fourth) {
    const double values[4]={first,second,third,fourth};
    for (unsigned i=0;i<4;++i) position[parent.nodes[i]]=values[i];
  }
  TL_SURFACE_HD sc::Q4NormalIntegrationInput Input() const {
    return {{{position,4,1,4},{velocity,4,3,1},&parent,1},
            {inverse,fixed,4,9},0,7,0,1,1,2};
  }
};
TL_SURFACE_HD inline sc::Q4IntegrationLimits Limits(double penetration=1) {
  sc::Q4IntegrationLimits limits;
  limits.force_error=1e-6*penetration;
  limits.energy_error=5e-9*penetration*penetration;
  return limits;
}

struct Oracle { long double force[4]{},potential=0; };
inline Oracle Uniform(long double d) {
  return {{d/4,d/4,d/4,d/4},d*d/2};
}
inline Oracle Bilinear(long double a,long double b,long double c,long double d) {
  return {{a/4+b/6+c/6+d/9,a/4+b/12+c/6+d/18,
           a/4+b/12+c/12+d/36,a/4+b/6+c/12+d/18},
          (a*a+a*b+a*c+a*d/2+b*b/3+c*c/3+d*d/9+b*c/2+b*d/3+c*d/3)/2};
}
inline Oracle Cut(long double a) {
  const long double b=1-a,s=b*b/4-b*b*b/12,l=b*b*b/12;
  return {{s,l,l,s},b*b*b/6};
}
inline Oracle Corner(long double e) {
  const long double e2=e*e,e3=e2*e,e4=e3*e,e5=e4*e;
  return {{e5/120,e4/24-e5/120,e3/6-e4/12+e5/120,e4/24-e5/120},e4/24};
}
inline Oracle Saddle() { return {{13.L/288,5.L/288,13.L/288,5.L/288},1.L/36}; }
}  // namespace q4_contact_test
