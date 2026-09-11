// SPDX-License-Identifier: MIT
#include "NativeValues.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cfloat>
#include <cmath>
namespace type25_mapped_test {
namespace spring=tl::fea::type25;
std::array<double,4> NativeStiffness(spring::SourceUnits u,const spring::Property& p,
    double length,bool active,unsigned mode) {
  const double j=u.mass_to_kg*u.length_to_m*u.length_to_m;
  const double t2=u.time_to_s*u.time_to_s;
  const double kt=u.mass_to_kg/t2,kr=j/t2;
  const double ct=u.mass_to_kg/u.time_to_s,cr=j/u.time_to_s;
  double property[10]{p.mass_kg/u.mass_to_kg,p.isotropic_inertia_kg_m2/j};
  for (unsigned channel=0;channel<4;++channel) {
    property[2+channel]=p.stiffness[channel]/(channel<2?kt:kr);
    property[6+channel]=p.damping[channel]/(channel<2?ct:cr);
  }
  const double working_length=length/u.length_to_m;
  const int off=active?1:0;
  double mass[3]{},inertia[3]{};
  int roots[2]{};
  if (mode) {
    mass[0]=.5; mass[1]=2; mass[2]=20;
    inertia[0]=.25; inertia[1]=4; inertia[2]=32;
  }
  if (mode==2) roots[0]=roots[1]=3;
  std::array<double,4> result;
  mapped_type25_native_stiffness(property,&working_length,&off,mass,inertia,roots,result.data());
  result[0]*=kt; result[1]*=kt;
  result[2]*=kr; result[3]*=kr;
  return result;
}
void CompareNative(const spring::Evaluation& actual,const spring::Evaluation& native,
    spring::SourceUnits u,const spring::Property& property,
    const spring::EndpointKinematics (&nodes)[2],double dt) {
  const auto a=type25_test::EvaluationValues(actual);
  const auto b=type25_test::EvaluationValues(native);
  double coordinate=actual.frame.length_m;
  for (const auto& node:nodes) coordinate=std::max({coordinate,std::abs(node.position.x),
      std::abs(node.position.y),std::abs(node.position.z)});
  // Same established TYPE25 oracle enclosure for SI/source-coordinate subtraction.
  // Only force/couple components use this geometric cancellation allowance.
  const double conversion=u.length_to_m==1?0:64*DBL_EPSILON*coordinate*
      (std::max(property.stiffness[0],property.stiffness[1])+
       std::max(property.damping[0],property.damping[1])/dt);
  for (std::size_t k=0;k<a.size();++k) {
    double allowance=0;
    if (k>=29 && k<32) allowance=conversion;
    if (k>=40 && k<52) allowance=((k-40)%6<3)?conversion:conversion*actual.frame.length_m;
    EXPECT_NEAR(a[k],b[k],allowance+5e-10*std::max({std::abs(a[k]),std::abs(b[k]),1e-20}))<<k;
  }
  EXPECT_EQ(actual.history.active,native.history.active);
}
}
