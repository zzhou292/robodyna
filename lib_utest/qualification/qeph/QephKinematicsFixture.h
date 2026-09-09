#pragma once
#include "QephStartupFixture.h"
#include "lib_src/elements/qeph/QephKinematics.h"
#include <iomanip>

namespace qeph_kinematics_test {
using namespace qeph_startup_test;
using tl::math::Matrix3;
// Frozen before Q3b execution. Native/host/device parity keeps the Q3a 2e-12
// coefficient, with explicit field dimensions below. Independent covariance
// retains Q1's separately qualified 2e-11 coefficient. No dynamics budget.
constexpr double kCovariance=2e-11;
inline Vec3 Multiply(Vec3 x,double a) { return {x.x*a,x.y*a,x.z*a}; }
inline Vec3 Cross(Vec3 a,Vec3 b) { return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x}; }
inline Vec3 Column(const Matrix3& f,unsigned c) { return {f.v[c],f.v[3+c],f.v[6+c]}; }
inline double Dot(Vec3 a,Vec3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
inline Vec3 Unit(Vec3 a) { return Multiply(a,1/Length(a)); }
inline port::PrescribedInterval Stationary(const port::ReferenceInput& input,double dt=.02) {
  port::PrescribedInterval result;
  for(unsigned n=0;n<4;++n) result.position_endpoint[n]=input.position[n];
  result.base_time=.125; result.dt=dt; result.sample_index=73; return result;
}
inline port::PrescribedInterval Rigid(const port::ReferenceInput& input,Vec3 omega,double dt) {
  auto result=Stationary(input,dt);
  const auto end=Rotation(Multiply(omega,dt)),mid=Rotation(Multiply(omega,.5*dt));
  for(unsigned n=0;n<4;++n) {
    result.position_endpoint[n]=Rotate(end,input.position[n]);
    result.velocity_midpoint[n]=Cross(omega,Rotate(mid,input.position[n]));
    result.omega_midpoint[n]=omega;
  }
  return result;
}
inline port::PrescribedInterval Reparameterize(port::PrescribedInterval input,unsigned shift,bool transform) {
  const auto base=input; const auto q=CommonRotation();
  for(unsigned n=0;n<4;++n) {
    const unsigned k=(n+shift)%4;
    input.position_endpoint[n]=base.position_endpoint[k];
    input.velocity_midpoint[n]=base.velocity_midpoint[k]; input.omega_midpoint[n]=base.omega_midpoint[k];
    if(transform) {
      const auto p=Rotate(q,input.position_endpoint[n]); input.position_endpoint[n]={p.x+1.25,p.y-.75,p.z+.5};
      input.velocity_midpoint[n]=Rotate(q,input.velocity_midpoint[n]);
      input.omega_midpoint[n]=Rotate(q,input.omega_midpoint[n]);
    }
  }
  return input;
}
inline port::PrescribedInterval Pattern(const port::ReferenceInput& input,unsigned pattern) {
  auto result=Stationary(input,1e-4);
  if(pattern==2) return Rigid(input,Unit({1,2,3}),.02);
  if(pattern==1) for(unsigned n=0;n<4;++n) {
    const auto p=input.position[n];
    result.velocity_midpoint[n]={.01*p.x+.02*p.y,-.015*p.y,.005*p.x};
    result.omega_midpoint[n]={.007*p.y,-.006*p.x,.004};
  }
  return result;
}
inline native::PrescribedInterval NativeInterval(const port::PrescribedInterval& input) {
  native::PrescribedInterval result;
  for(unsigned n=0;n<4;++n) {
    result.position_endpoint[n]=input.position_endpoint[n]; result.velocity_midpoint[n]=input.velocity_midpoint[n];
    result.omega_midpoint[n]=input.omega_midpoint[n];
  }
  result.base_time=input.base_time; result.dt=input.dt; result.sample_index=input.sample_index; return result;
}
inline double LengthScale(const port::PrescribedInterval& input) {
  double l=0; for(unsigned n=1;n<4;++n) l=std::max(l,Length(Difference(input.position_endpoint[n],input.position_endpoint[0])));
  return l;
}
inline double RateScale(const port::PrescribedInterval& input) {
  // A declared 1/s unit floor also permits independent exact-zero checks.
  double r=1; const double l=LengthScale(input);
  for(unsigned n=0;n<4;++n) {
    r=std::max(r,Length(input.omega_midpoint[n]));
    r=std::max(r,Length(Difference(input.velocity_midpoint[n],input.velocity_midpoint[0]))/l);
  }
  return r;
}
inline void Field(double a,double b,double dimension,double coefficient=kRoundoff,
                  const char* name="independent_scalar",unsigned index=0) {
  SCOPED_TRACE(name);
  SCOPED_TRACE(index);
  ASSERT_TRUE(std::isfinite(a)); ASSERT_TRUE(std::isfinite(b)); ASSERT_GT(dimension,0.);
  EXPECT_LE(std::abs(a-b),coefficient*(dimension+std::abs(b)))
      <<std::setprecision(17)<<"actual="<<a<<" reference="<<b<<" dimension="<<dimension;
}
inline void Field(Vec3 a,Vec3 b,double dimension,double coefficient=kRoundoff,
                  const char* name="independent_vector",unsigned index=0) {
  Field(a.x,b.x,dimension,coefficient,name,3*index);
  Field(a.y,b.y,dimension,coefficient,name,3*index+1);
  Field(a.z,b.z,dimension,coefficient,name,3*index+2);
}
template<class Expected>
inline void Agreement(const port::Kinematics& a,const Expected& b,const port::PrescribedInterval& input,
                      bool compare_frame=true,double coefficient=kRoundoff) {
  const double l=LengthScale(input),r=RateScale(input);
  EXPECT_EQ(a.planar,b.planar); EXPECT_EQ(a.base_time,b.base_time); EXPECT_EQ(a.dt,b.dt); EXPECT_EQ(a.sample_index,b.sample_index);
  if(compare_frame) for(unsigned i=0;i<9;++i) Field(a.frame.v[i],b.frame.v[i],1,coefficient,"frame",i);
  Field(a.area,b.area,l*l,coefficient,"area");
  Field(a.reciprocal_area,b.reciprocal_area,1/(l*l),coefficient,"reciprocal_area");
  Field(a.characteristic_length,b.characteristic_length,l,coefficient,"characteristic_length");
  Field(a.raw_warpage_abs,b.raw_warpage_abs,l,coefficient,"raw_warpage_abs");
  Field(a.effective_warpage,b.effective_warpage,l,coefficient,"effective_warpage");
  for(unsigned n=0;n<2;++n) Field(a.nodal_factors[n],b.nodal_factors[n],1,coefficient,"nodal_factors",n);
  for(unsigned n=0;n<4;++n) {
    Field(a.local_position[n],b.local_position[n],l,coefficient,"local_position",n);
    Field(a.local_normals[n],b.local_normals[n],1,coefficient,"local_normals",n);
    // DI/DB are mixed source-coordinate projection coefficients, with a native
    // additive 4 beside squared lengths. Declare unit numerical floors in SI;
    // do not mislabel these as uniform inverse-length tensors.
    Field(a.projection_columns[n],b.projection_columns[n],1,coefficient,"projection_columns",n);
  }
  for(unsigned i=0;i<6;++i) Field(a.projection_inverse[i],b.projection_inverse[i],1,coefficient,"projection_inverse",i);
  for(unsigned i=0;i<8;++i) {
    Field(a.projected_omega[i],b.projected_omega[i],r,coefficient,"projected_omega",i);
    Field(a.regular_rate[i],b.regular_rate[i],i<5?r:r/l,coefficient,"regular_rate",i);
  }
  for(unsigned i=0;i<6;++i) Field(a.hourglass_rate[i],b.hourglass_rate[i],i==2||i==3?r:r*l,coefficient,"hourglass_rate",i);
}
inline double NativeRectangleLength(double a,double b) {
  // Independent rectangle specialization with exact hexadecimal values of
  // the donor default-REAL literals, evaluated in long double. It neither
  // calls port geometry nor reads native output. Decimal binary64 literals
  // are observably different, as retained Q3b first-run evidence shows.
  constexpr long double multiplier=0x1.b4dd3p+1L,threshold=0x1.6a0902p-1L;
  constexpr long double intercept=0x1.8f5c28p-1L,cubic=0x1.c28f5cp-3L;
  const long double x=a,y=b,ratio=std::max(x,y)/std::min(x,y);
  const long double factor1=1+std::min(.5L,.25L*(ratio-1));
  const long double s=multiplier*(1-threshold),factor2=intercept+cubic*s*s*s;
  return static_cast<double>(4*x*y/std::sqrt(2*factor1*factor2*1.25L*(x*x+y*y)));
}
inline void ZeroRates(const port::Kinematics& output,const port::PrescribedInterval& input) {
  const double l=LengthScale(input),r=RateScale(input);
  for(unsigned i=0;i<8;++i) Field(output.regular_rate[i],0,i<5?r:r/l);
  for(unsigned i=0;i<6;++i) Field(output.hourglass_rate[i],0,i==2||i==3?r:r*l);
}
inline std::array<double,8> PlanarRigidTruth(Vec3 unit_omega,double h) {
  // Independent closed-form gradient oracle retained from the qualified Q1
  // characterization: H=Omega exp(-h Omega/2). No native nodal packing call.
  const double w[]{unit_omega.x,unit_omega.y,unit_omega.z},d=.5*h;
  const double spin[3][3]{{0,-w[2],w[1]},{w[2],0,-w[0]},{-w[1],w[0],0}};
  double gradient[3][3]{},corrected[2][3]{};
  for(unsigned i=0;i<3;++i) for(unsigned j=0;j<3;++j)
    gradient[i][j]=std::cos(d)*spin[i][j]+std::sin(d)*((i==j?1.:0.)-w[i]*w[j]);
  for(unsigned j=0;j<3;++j) {
    corrected[0][j]=gradient[0][j]-d*gradient[2][0]*gradient[2][j]-d*gradient[1][0]*gradient[1][j];
    corrected[1][j]=gradient[1][j]-d*gradient[2][1]*gradient[2][j]-d*gradient[0][1]*gradient[0][j];
  }
  return {{corrected[0][0],corrected[1][1],corrected[0][1]+corrected[1][0],
            gradient[2][0]+w[1],gradient[2][1]-w[0],0,0,0}};
}
inline void RigidTruth(const port::Kinematics& output,const port::PrescribedInterval& input,Vec3 omega) {
  const auto truth=PlanarRigidTruth(omega,input.dt); const double l=LengthScale(input);
  for(unsigned i=0;i<8;++i) Field(output.regular_rate[i],truth[i],i<5?1:1/l);
  for(unsigned i=0;i<6;++i) Field(output.hourglass_rate[i],0,i==2||i==3?1:l);
}
inline port::PrescribedInterval Affine(const port::ReferenceInput& ref,unsigned mode) {
  auto in=Stationary(ref,1e-6); constexpr double alpha=.04; const double beta=alpha/Scale(ref);
  for(unsigned n=0;n<4;++n) {
    const auto p=ref.position[n];
    if(mode==0) in.velocity_midpoint[n].x=alpha*p.x;
    if(mode==1) in.velocity_midpoint[n].y=alpha*p.y;
    if(mode==2) in.velocity_midpoint[n].x=alpha*p.y;
    if(mode==3) in.velocity_midpoint[n].z=alpha*p.x;
    if(mode==4) in.velocity_midpoint[n].z=alpha*p.y;
    if(mode==5) in.omega_midpoint[n].y=beta*p.x;
    if(mode==6) in.omega_midpoint[n].x=-beta*p.y;
    if(mode==7) in.omega_midpoint[n].y=beta*p.y;
  }
  return in;
}
inline void AffineTruth(const port::Kinematics& result,const port::ReferenceInput& ref,unsigned mode) {
  std::array<double,8> expected{}; constexpr double alpha=.04,h=1e-6;
  expected[mode]=mode<5?alpha:alpha/Scale(ref);
  if(mode==2||mode==4) expected[1]=-.5*h*alpha*alpha;
  if(mode==3) expected[0]=-.5*h*alpha*alpha;
  const auto interval=Affine(ref,mode); const double l=LengthScale(interval);
  for(unsigned i=0;i<8;++i) Field(result.regular_rate[i],expected[i],i<5?1:1/l);
  // Pure omega_y=beta*y has the native transverse twisting hourglass value
  // 4*A*beta on a rectangle; affine nodal rotations do not imply all HG zero.
  const double area=4*ref.position[1].x*ref.position[2].y;
  for(unsigned i=0;i<6;++i) {
    const double hg=mode==7&&i==4?4*area*alpha/Scale(ref):0;
    Field(result.hourglass_rate[i],hg,i==2||i==3?1:l);
  }
}
inline port::PrescribedInterval InvalidInterval(unsigned kind) {
  auto in=Stationary(Case(1));
  if(kind==0) in.dt=0;
  if(kind==1) in.base_time=1e30; // finite but nonadvancing represented endpoint
  if(kind==2) in.position_endpoint[3].z=std::numeric_limits<double>::quiet_NaN();
  if(kind==3) in.position_endpoint[3]=in.position_endpoint[0];
  if(kind==4) in.omega_midpoint[3].x=std::numeric_limits<double>::infinity();
  if(kind==5) {
    in.velocity_midpoint[0].x=std::numeric_limits<double>::max();
    in.velocity_midpoint[2].x=-std::numeric_limits<double>::max(); // late difference overflow
  }
  if(kind==6) in.dt=std::numeric_limits<double>::max(),in.base_time=in.dt;
  if(kind==7) in.position_endpoint[2]={-.2,0,0};
  return in;
}
inline void CorruptReference(port::ReferenceData& ref,unsigned kind) {
  if(kind==0) ref.prepared=false;
  if(kind==1) ref.input.density=std::numeric_limits<double>::quiet_NaN();
  if(kind==2) ref.nodal_mass[3]=0;
  if(kind==3) ref.frame.v[8]=0;
  if(kind==4) ref.derivative_y[3]=std::numeric_limits<double>::infinity();
}
} // namespace qeph_kinematics_test
