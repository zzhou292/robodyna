#include "GroupTestSupport.h"
#include <algorithm>
#include <limits>

extern "C" {
void nodal_rigid_native_inertia(const int*,const double*,const double*,const double*,const double*,const double*,double*);
void nodal_rigid_native_ispher2(const double*,double*,int*);
void nodal_rigid_native_wrench(const int*,const double*,const double*,const double*,const double*,double*);
void nodal_rigid_native_gyro(const double*,const double*,const double*,double*);
}
namespace rigid_test {
TEST(NodalRigidGroupNative,FullTensorUsesPinnedNativeMemberMassAndInertiaArithmetic) {
  auto members=Members(); fe::NodalRigidGroupInput group{1001,2001,members.data(),members.size()};
  fe::NodalRigidGroupModel model; ASSERT_TRUE(model.Initialize(Input(&group,1)));
  const auto& g=model.groups()[0]; const int count=4;
  double position[12],mass[4],inertia[4],center[3]{g.center.x,g.center.y,g.center.z};
  for(unsigned i=0;i<4;++i) {
    for(unsigned a=0;a<3;++a) position[3*i+a]=Get(members[i].position,a);
    mass[i]=members[i].mass_kg; inertia[i]=members[i].total_inertia_kg_m2;
  }
  double initial[9]{};
  // The generated primary is separate donor epsilon data, not a physical member.
  const auto r=rigid::detail::Subtract(g.generated_primary_position,g.center);
  const double square=r.x*r.x+r.y*r.y+r.z*r.z;
  for(unsigned a=0;a<3;++a) for(unsigned b=0;b<3;++b)
    initial[3*a+b]=(a==b?g.regularization.primary_isotropic_inertia_kg_m2:0)+
      g.regularization.primary_mass_kg*((a==b?square:0)-Get(r,a)*Get(r,b));
  double native[9]; nodal_rigid_native_inertia(&count,position,center,mass,inertia,initial,native);
  for(unsigned i=0;i<9;++i) EXPECT_NEAR(g.raw_tensor.v[i],native[i],2e-16);
}
TEST(NodalRigidGroupNative,CorrectionMatchesPinnedStrictThresholdAndIndependentEigenAxes) {
  const Vec3 cases[]{{1,2,1000},{std::nextafter(1.,0.),2,1000},{std::nextafter(1.,2.),2,1000},
                    {1.00000002,2,1000},{0,2,1000},{-.01,2,1000},{.001,1000,.0001},{1,1,1}};
  for(const auto raw:cases) {
    const double input[]{raw.x,raw.y,raw.z}; double native[3]; int reached=0;
    nodal_rigid_native_ispher2(input,native,&reached);
    rigid::PrincipalCorrection actual;
    ASSERT_EQ(rigid::CorrectPrincipalInertia(raw,actual),rigid::MathStatus::Success);
    for(unsigned i=0;i<3;++i) EXPECT_DOUBLE_EQ(Get(actual.effective,i),native[i]);
    EXPECT_EQ(actual.threshold_reached,reached!=0);
  }
}
TEST(NodalRigidGroupNative,WrenchMatchesPinnedOffCenterForceAndNodalCoupleTransfer) {
  const int count=3;
  const Vec3 x[]{{1,2,3},{-.5,.4,.9},{.3,-.7,.2}},f[]{{2,3,5},{7,-4,2},{-1,9,3}},c[]{{.1,.2,.3},{-.4,.8,.2},{.3,-.1,.6}};
  const Vec3 center{.25,.1,-.5};
  double positions[9],forces[9],couples[9],origin[]{center.x,center.y,center.z},native[6];
  for(unsigned i=0;i<3;++i) for(unsigned a=0;a<3;++a) {
    positions[3*i+a]=Get(x[i],a); forces[3*i+a]=Get(f[i],a); couples[3*i+a]=Get(c[i],a);
  }
  nodal_rigid_native_wrench(&count,positions,origin,forces,couples,native);
  rigid::Wrench actual; ASSERT_EQ(rigid::AggregateWrench(center,x,f,c,3,actual),rigid::MathStatus::Success);
  Near(actual.force,{native[0],native[1],native[2]},1e-14);
  Near(actual.couple,{native[3],native[4],native[5]},1e-14);
}
TEST(NodalRigidGroupNative,AnisotropicGyroMatchesPinnedBodyFramePseudoMomentCancellation) {
  const auto frame=Frame(); const Vec3 w{.7,-1.2,.4},torque{2,-3,1};
  // Explicit cyclic inverse permutation, independent of production matrix helper.
  const double principal[]{2,3,5},omega[]{w.y,w.z,w.x},local_torque[]{torque.y,torque.z,torque.x};
  double native[3]; nodal_rigid_native_gyro(principal,omega,local_torque,native);
  Vec3 actual;
  ASSERT_EQ(rigid::AngularAcceleration(frame,w,torque,actual),rigid::MathStatus::Success);
  Near(actual,{native[2],native[0],native[1]},2e-15);
}
} // namespace rigid_test
