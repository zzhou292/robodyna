#pragma once
#include "lib_src/elements/qeph/QephStartup.h"
#include "lib_utest/qualification/native/qeph/QephReference.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <type_traits>

namespace qeph_startup_test {
namespace port=tl::fea::qeph;
namespace native=tl::qualification::qeph;
using tl::math::Vec3;
// Frozen before Q3a execution, including source-scale cases. Error uses its
// actual dimension: dimensionless frame, L coordinates/derivatives, L^2 area,
// and the independent mass or inertia partition itself for mass quantities.
constexpr double kRoundoff=2e-12;
constexpr unsigned kOrthogonalCases=6;
constexpr unsigned kCases=9;
inline double Length(Vec3 x) { return std::hypot(x.x,x.y,x.z); }
inline Vec3 Difference(Vec3 a,Vec3 b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
inline tl::math::Quaternion Rotation(Vec3 spin) {
  const double delta[]{spin.x,spin.y,spin.z}; tl::math::Quaternion q;
  EXPECT_TRUE(tl::math::IncrementWorldRotation({},delta,q)); return q;
}
inline Vec3 Rotate(tl::math::Quaternion q,Vec3 x) {
  const auto y=tl::math::Product(tl::math::Product(q,{0,x.x,x.y,x.z}),{q.w,-q.x,-q.y,-q.z});
  return {y.x,y.y,y.z};
}
inline tl::math::Quaternion CommonRotation() {
  return tl::math::Product(Rotation({.4,-.7,.3}),Rotation({-.2,.25,1.1}));
}
inline port::ReferenceInput Case(unsigned index) {
  port::ReferenceInput input;
  if (index<kOrthogonalCases) {
    const double a=index<2?1:(index<4?.005:.01),b=index<2?.5:a;
    const double warp=index%2?(index<2?.05:.08*a):0;
    input.position[0]={-a,-b,warp}; input.position[1]={a,-b,-warp};
    input.position[2]={a,b,warp}; input.position[3]={-a,b,-warp};
  } else {
    const double scale=index==6?1:(index==7?.005:.01);
    input.position[0]={0,0,0}; input.position[1]={2*scale,0,0};
    input.position[2]={2.5*scale,scale,0}; input.position[3]={.5*scale,scale,0};
  }
  const unsigned ids[]{17,3,1001,51};
  for (unsigned n=0;n<4;++n) input.node_ids[n]=ids[n];
  if (index<2||index==6) { input.density=10; input.young_modulus=2e6; input.poisson_ratio=.25; input.thickness=.1; }
  return input;
}
inline port::ReferenceInput Reparameterize(port::ReferenceInput input,unsigned shift,bool transform) {
  const auto original=input;
  const auto q=CommonRotation();
  for (unsigned n=0;n<4;++n) {
    input.position[n]=original.position[(n+shift)%4]; input.node_ids[n]=original.node_ids[(n+shift)%4];
    if (transform) {
      auto p=Rotate(q,input.position[n]); input.position[n]={p.x+1.25,p.y-.75,p.z+.5};
    }
  }
  return input;
}
inline native::ReferenceInput NativeInput(const port::ReferenceInput& input) {
  native::ReferenceInput result;
  for (unsigned n=0;n<4;++n) { result.position[n]=input.position[n]; result.node_ids[n]=input.node_ids[n]; }
  result.density=input.density; result.young_modulus=input.young_modulus;
  result.poisson_ratio=input.poisson_ratio; result.thickness=input.thickness;
  return result;
}
inline double Scale(const port::ReferenceInput& input) {
  double scale=0;
  for (unsigned n=1;n<4;++n) scale=std::max(scale,Length(Difference(input.position[n],input.position[0])));
  return scale;
}
inline void Near(double actual,double expected,double dimension) {
  ASSERT_TRUE(std::isfinite(actual)); ASSERT_TRUE(std::isfinite(expected)); ASSERT_GT(dimension,0.);
  EXPECT_LE(std::abs(actual-expected),kRoundoff*(dimension+std::abs(expected)));
}
inline void Near(Vec3 actual,Vec3 expected,double dimension) {
  Near(actual.x,expected.x,dimension); Near(actual.y,expected.y,dimension); Near(actual.z,expected.z,dimension);
}
inline void SameInput(const port::ReferenceInput& actual,const port::ReferenceInput& expected) {
  for (unsigned n=0;n<4;++n) {
    EXPECT_EQ(actual.node_ids[n],expected.node_ids[n]);
    EXPECT_DOUBLE_EQ(actual.position[n].x,expected.position[n].x);
    EXPECT_DOUBLE_EQ(actual.position[n].y,expected.position[n].y);
    EXPECT_DOUBLE_EQ(actual.position[n].z,expected.position[n].z);
  }
  EXPECT_DOUBLE_EQ(actual.density,expected.density); EXPECT_DOUBLE_EQ(actual.young_modulus,expected.young_modulus);
  EXPECT_DOUBLE_EQ(actual.poisson_ratio,expected.poisson_ratio); EXPECT_DOUBLE_EQ(actual.thickness,expected.thickness);
}
inline void Agreement(const port::ReferenceData& actual,const native::ReferenceData& expected) {
  ASSERT_TRUE(actual.prepared); const double length=Scale(actual.input);
  for (unsigned n=0;n<4;++n) {
    EXPECT_EQ(actual.input.node_ids[n],expected.input.node_ids[n]);
    Near(actual.derivative_x[n],expected.derivative_x[n],length);
    Near(actual.derivative_y[n],expected.derivative_y[n],length);
    Near(actual.local_position[n],expected.local_position[n],length);
    Near(actual.nodal_mass[n],expected.nodal_mass[n],expected.nodal_mass[n]);
    Near(actual.physical_inertia[n],expected.physical_inertia[n],expected.physical_inertia[n]);
    Near(actual.added_inertia[n],expected.added_inertia[n],expected.added_inertia[n]);
    Near(actual.isotropic_inertia[n],expected.isotropic_inertia[n],expected.isotropic_inertia[n]);
  }
  for (unsigned i=0;i<9;++i) Near(actual.frame.v[i],expected.frame.v[i],1.);
  Near(actual.area,expected.area,length*length);
}
inline void SameResult(const port::ReferenceData& a,const port::ReferenceData& b) {
  ASSERT_TRUE(a.prepared); ASSERT_TRUE(b.prepared); SameInput(a.input,b.input);
  const double length=Scale(a.input);
  for (unsigned n=0;n<4;++n) {
    Near(a.derivative_x[n],b.derivative_x[n],length); Near(a.derivative_y[n],b.derivative_y[n],length);
    Near(a.local_position[n],b.local_position[n],length);
    Near(a.nodal_mass[n],b.nodal_mass[n],b.nodal_mass[n]);
    Near(a.physical_inertia[n],b.physical_inertia[n],b.physical_inertia[n]);
    Near(a.added_inertia[n],b.added_inertia[n],b.added_inertia[n]);
    Near(a.isotropic_inertia[n],b.isotropic_inertia[n],b.isotropic_inertia[n]);
  }
  for (unsigned i=0;i<9;++i) Near(a.frame.v[i],b.frame.v[i],1.);
  Near(a.area,b.area,length*length);
}
template<class T> auto Bytes(const T& object) {
  static_assert(std::is_trivially_copyable_v<T>);
  std::array<unsigned char,sizeof(T)> result; std::memcpy(result.data(),&object,sizeof(T)); return result;
}
// A square with native DET=16*half_side^2, kept near the origin. This explicit
// fixture tests the measured determinant domain; it is not source vehicle scale.
inline port::ReferenceInput Threshold(double half_side) {
  auto input=Case(0);
  input.position[0]={-half_side,-half_side,0}; input.position[1]={half_side,-half_side,0};
  input.position[2]={half_side,half_side,0}; input.position[3]={-half_side,half_side,0};
  return input;
}
inline port::ReferenceInput Failure(unsigned kind) {
  auto input=Case(0);
  if (kind==0) input.node_ids[3]=input.node_ids[0];
  if (kind==1) input.position[3].z=std::numeric_limits<double>::quiet_NaN();
  if (kind==2) input.poisson_ratio=-.1;
  if (kind==3) input.position[3]=input.position[0];
  if (kind==4) input.thickness=4, input.density=std::numeric_limits<double>::max();
  if (kind==5) input.thickness=1e-110; // Mass and area partition positive, t^3 inertia underflows.
  if (kind==6) input=Threshold(std::nextafter(2.5e-11,std::numeric_limits<double>::infinity()));
  if (kind==7) { input.position[2]={-.2,0,0}; } // Concave projected corner.
  if (kind==8) for (auto& p:input.position) { p.x*=1e200; p.y*=1e200; }
  return input;
}
}  // namespace qeph_startup_test
