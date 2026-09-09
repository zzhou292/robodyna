#pragma once

#include "lib_utest/q4_contact_integration_fixture.h"
#include "lib_src/collision/Q4RectangularIntegration.h"
#include <cmath>

namespace q4_rectangular_test {
namespace sc=tlfea::contact;
using q4_contact_test::Fixture;
using q4_contact_test::Limits;
using q4_contact_test::Oracle;
TL_SURFACE_HD inline void Configure(unsigned kind,Fixture& fixture,sc::Q4IntegrationLimits& limits) {
  limits=Limits();
  if (kind==0) fixture.Gaps(.625,-.375,-.375,.625);
  if (kind==1) { const double e=1./64; fixture.Gaps(e-2,e-1,e,e-1); limits=Limits(e); }
  if (kind==2) fixture.Gaps(1,-1,1,-1);
  if (kind==3) { const double d=::ldexp(1.,-40); fixture.Gaps(.625+d,-.375+d,-.375,.625); }
  if (kind==4) fixture.Gaps(1.875,1.5,1,1.25);
}
// Independent unit-square integration for g(s,t)=s-a+d*t, 0<a-d*t<1.
// Integrating first in s from a-d*t to1 gives the positive-side first moments
// b(t)^2/2-b(t)^3/6 and b(t)^3/6; integrate those polynomials with t or1-t.
// No dyadic partition, Gauss sample, interval moments or contact law is called.
inline Oracle ObliqueCut(long double a,long double d) {
  const long double b=1-a;
  const long double square[4]={b*b,2*b*d,d*d,0};
  const long double cube[4]={b*b*b,3*b*b*d,3*b*d*d,d*d*d};
  Oracle result;
  for (unsigned p=0;p<4;++p) {
    const long double high=square[p]/2-cube[p]/6,low=cube[p]/6;
    const long double all=1.L/(p+1),t=1.L/(p+2);
    result.force[0]+=high*t; result.force[1]+=low*t;
    result.force[2]+=low*(all-t); result.force[3]+=high*(all-t);
    result.potential+=cube[p]*all/6;
  }
  return result;
}
inline Oracle Expected(unsigned kind) {
  if (kind==0) return q4_contact_test::Cut(.375L);
  if (kind==1) return q4_contact_test::Corner(1.L/64);
  if (kind==2) return q4_contact_test::Saddle();
  if (kind==3) return ObliqueCut(.375L,std::ldexp(1.L,-40));
  return q4_contact_test::Bilinear(1,.25,.5,.125);
}
} // namespace q4_rectangular_test
