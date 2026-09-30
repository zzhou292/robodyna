#pragma once
// Test-only continuous affine-gradient oracle, evaluated in long double.
// The simplified angular shear polynomials below are independent contractions
// for a local triangle (0,0),(a,0),(b,c), not calls to native rate routines.
#include "T3Kinematics.h"
#include "T3StartupTestOracle.h"
#include "lib_src/math/Quaternion.h"
#include <algorithm>
#include <cstring>
#include <stdexcept>

namespace tl::qualification::t3::kinematic_test {
using test::Wide;
inline Wide Add(Wide a,Wide b) { return {a[0]+b[0],a[1]+b[1],a[2]+b[2]}; }
inline Wide Scale(Wide a,long double s) { return {s*a[0],s*a[1],s*a[2]}; }
inline Wide Cross(Wide a,Wide b) { return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]}; }
inline Wide Widen(Vec3 a) { return {a.x,a.y,a.z}; }
inline Vec3 Narrow(Wide a) { return {double(a[0]),double(a[1]),double(a[2])}; }
inline Wide Unit(Wide a) { return Scale(a,1/std::sqrt(test::Dot(a,a))); }
template<class T> auto Bytes(const T& a) {
  std::array<unsigned char,sizeof(T)> bytes; std::memcpy(bytes.data(),&a,sizeof(a)); return bytes;
}
inline ReferenceInput Triangle(double length=1) {
  ReferenceInput r; r.position={{{0,0,0},{2*length,0,0},{.375*length,length,0}}};
  r.node_ids={{101,8,45001}}; return r;
}
inline Reference MakeReference(const ReferenceInput& r) {
  Reference out;
  if(Initialize(r,out)!=Status::kSuccess)throw std::runtime_error("Native T3 test reference rejected");
  return out;
}
inline PrescribedInterval Interval(const ReferenceInput& r,double h=.01) {
  PrescribedInterval in; in.position=r.position; in.dt=h; return in;
}
inline tl::math::Quaternion Rotation() {
  tl::math::Quaternion q; const double w[]{.41,-.73,.23};
  if(!tl::math::IncrementWorldRotation({},w,q))throw std::runtime_error("Test rotation failed");
  return q;
}
inline Vec3 Rotate(tl::math::Quaternion q,Vec3 v) {
  const auto a=tl::math::Product(tl::math::Product(q,{0,v.x,v.y,v.z}),{q.w,-q.x,-q.y,-q.z});
  return {a.x,a.y,a.z};
}
struct Oracle {
  std::array<Wide,3> basis{},local{},local_velocity{},local_omega{};
  std::array<long double,8> raw{},normalized{};
  std::array<long double,3> differences{};
  long double area=0,length=0,scale_length=0,velocity=0,omega=0;
};
inline Oracle Independent(const PrescribedInterval& in) {
  Oracle out;
  const auto edge=test::Difference(in.position[1],in.position[0]);
  const auto edge2=test::Difference(in.position[2],in.position[0]);
  out.basis[0]=Unit(edge); out.basis[2]=Unit(Cross(edge,edge2));
  out.basis[1]=Unit(Cross(out.basis[2],out.basis[0]));
  for(unsigned n=0;n<3;++n) {
    for(unsigned k=0;k<3;++k) {
      out.local[n][k]=test::Dot(test::Difference(in.position[n],in.position[0]),out.basis[k]);
      out.local_velocity[n][k]=test::Dot(Widen(in.velocity[n]),out.basis[k]);
      out.local_omega[n][k]=test::Dot(Widen(in.angular_velocity[n]),out.basis[k]);
    }
    out.scale_length=std::max(out.scale_length,std::sqrt(test::Dot(
        test::Difference(in.position[(n+1)%3],in.position[n]),test::Difference(in.position[(n+1)%3],in.position[n]))));
    out.velocity=std::max(out.velocity,std::sqrt(test::Dot(Widen(in.velocity[n]),Widen(in.velocity[n]))));
    out.omega=std::max(out.omega,std::sqrt(test::Dot(Widen(in.angular_velocity[n]),Widen(in.angular_velocity[n]))));
  }
  const long double a=out.local[1][0],b=out.local[2][0],c=out.local[2][1];
  out.area=test::CrossNorm(edge,edge2)/2; out.length=2*out.area/out.scale_length;
  // Exact gradients of the linear interpolation, independent of PX/PY source arrays.
  const std::array<std::array<long double,2>,3> gradient{{{{-1/a,(b-a)/(a*c)}},{{1/a,-b/(a*c)}},{{0,1/c}}}};
  long double dv[3][2]{},dw[3][2]{};
  for(unsigned n=0;n<3;++n)for(unsigned k=0;k<3;++k)for(unsigned d=0;d<2;++d) {
    dv[k][d]+=out.local_velocity[n][k]*gradient[n][d];
    dw[k][d]+=out.local_omega[n][k]*gradient[n][d];
  }
  const long double h=in.dt;
  const long double xx=dv[0][0]-.5L*h*(dv[2][0]*dv[2][0]+dv[1][0]*dv[1][0]);
  const long double yy=dv[1][1]-.5L*h*(dv[2][1]*dv[2][1]+dv[0][1]*dv[0][1]);
  const long double xy=dv[0][1]+dv[1][0]-.5L*h*(2*dv[2][0]*dv[2][1]+dv[1][0]*dv[1][1]+dv[0][1]*dv[0][0]);
  // Geometry-dependent native three-node transverse interpolation, simplified
  // in terms of affine angular gradients. It is not mean nodal rotation.
  const long double zx=dv[2][0]+out.local_omega[0][1]+a/2*dw[1][0]+c/6*(dw[0][0]+dw[1][1]);
  const long double yz=dv[2][1]-out.local_omega[0][0]-(a+4*b)/6*dw[0][0]-c/2*dw[0][1]
      +b*(b-a)/(2*c)*dw[1][0]+(2*b-a)/6*dw[1][1];
  out.normalized={xx,yy,xy,yz,zx,dw[1][0],-dw[0][1],dw[1][1]-dw[0][0]};
  for(unsigned k=0;k<8;++k)out.raw[k]=out.area*out.normalized[k];
  std::array<Wide,3> corrected=out.local_velocity;
  for(unsigned n=0;n<3;++n) {
    const auto v=out.local_velocity[n];
    corrected[n][0]=v[0]-.5L*h*(v[2]*dv[2][0]+v[1]*dv[1][0]);
    corrected[n][1]=v[1]-.5L*h*(v[2]*dv[2][1]+v[0]*dv[0][1]);
  }
  out.differences={corrected[0][0]-corrected[2][0],corrected[1][0]-corrected[2][0],corrected[0][1]-corrected[1][1]};
  return out;
}
inline void Check(const Kinematics& actual,const PrescribedInterval& in) {
  ASSERT_TRUE(actual.valid); const auto e=Independent(in); const auto L=e.scale_length;
  for(unsigned j=0;j<3;++j)for(unsigned i=0;i<3;++i)test::Near(actual.frame.v[3*i+j],e.basis[j][i],1);
  for(unsigned n=0;n<3;++n)for(unsigned j=0;j<3;++j) {
    const auto& v=actual.local_position[n];
    test::Near(j==0?v.x:j==1?v.y:v.z,e.local[n][j],L);
  }
  test::Near(actual.area,e.area,L*L); test::Near(actual.characteristic_length,e.length,L);
  EXPECT_EQ(actual.area_scale,1);
  const long double a=e.local[1][0],b=e.local[2][0],c=e.local[2][1];
  test::Near(actual.derivative[0],-c/2,L); test::Near(actual.derivative[1],(b-a)/2,L); test::Near(actual.derivative[2],-b/2,L);
  const long double velocity_scale=e.velocity+L*e.omega;
  for(unsigned k=0;k<8;++k) {
    SCOPED_TRACE(k);
    const long double scale=k<5?L*velocity_scale:L*e.omega;
    test::Near(actual.raw_rate[k],e.raw[k],scale);
    test::Near(actual.normalized_rate[k],e.normalized[k],scale/e.area);
  }
  for(unsigned k=0;k<3;++k)test::Near(actual.corrected_velocity_difference[k],e.differences[k],velocity_scale);
  EXPECT_EQ(actual.base_time,in.base_time); EXPECT_EQ(actual.position_time,in.base_time+in.dt);
  EXPECT_EQ(actual.velocity_time,in.base_time+.5*in.dt); EXPECT_EQ(actual.dt,in.dt); EXPECT_EQ(actual.sample_index,in.sample_index);
}
}  // namespace tl::qualification::t3::kinematic_test
