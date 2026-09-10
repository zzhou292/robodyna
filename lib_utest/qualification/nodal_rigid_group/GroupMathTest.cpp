#include "GroupTestSupport.h"
#include <limits>

namespace rigid_test {
TEST(NodalRigidGroupMath,SourceCorrectionHasExactStrictAxisBoundary) {
  rigid::PrincipalCorrection equal;
  ASSERT_EQ(rigid::CorrectPrincipalInertia({1,2,1000},equal),rigid::MathStatus::Success);
  EXPECT_TRUE(equal.threshold_reached); EXPECT_FALSE(equal.changed); Near(equal.effective,{1,2,1000},0);
  rigid::PrincipalCorrection promoted;
  ASSERT_EQ(rigid::CorrectPrincipalInertia({1.00000002,2,1000},promoted),rigid::MathStatus::Success);
  EXPECT_TRUE(promoted.threshold_reached); EXPECT_FALSE(promoted.changed);
  rigid::PrincipalCorrection below;
  ASSERT_EQ(rigid::CorrectPrincipalInertia({std::nextafter(1.,0.),2,1000},below),rigid::MathStatus::Success);
  EXPECT_TRUE(below.changed); EXPECT_DOUBLE_EQ(below.effective.x,std::nextafter(1.,0.)+100);
  rigid::PrincipalCorrection zero;
  ASSERT_EQ(rigid::CorrectPrincipalInertia({0,2,1000},zero),rigid::MathStatus::Success);
  EXPECT_DOUBLE_EQ(zero.effective.x,100);
  auto saved=zero;
  EXPECT_EQ(rigid::CorrectPrincipalInertia({-1,-2,-3},zero),rigid::MathStatus::InvalidInput);
  Near(zero.effective,saved.effective,0);
}
TEST(NodalRigidGroupMath,WrenchReductionSatisfiesIndependentLongDoubleVirtualWork) {
  const Vec3 x[]{{1,2,3},{-.5,.4,.9},{.3,-.7,.2}},f[]{{2,3,5},{7,-4,2},{-1,9,3}},c[]{{.1,.2,.3},{-.4,.8,.2},{.3,-.1,.6}};
  const Vec3 center{.25,.1,-.5},v{.3,-.7,.8},omega{.9,-.2,.6};
  rigid::Wrench out; ASSERT_EQ(rigid::AggregateWrench(center,x,f,c,3,out),rigid::MathStatus::Success);
  long double member_power=0;
  for(unsigned i=0;i<3;++i) {
    const long double r[]{x[i].x-center.x,x[i].y-center.y,x[i].z-center.z};
    const long double velocity[]{v.x+omega.y*r[2]-omega.z*r[1],v.y+omega.z*r[0]-omega.x*r[2],v.z+omega.x*r[1]-omega.y*r[0]};
    for(unsigned a=0;a<3;++a) member_power+=velocity[a]*Get(f[i],a)+static_cast<long double>(Get(omega,a))*Get(c[i],a);
  }
  const long double group_power=static_cast<long double>(out.force.x)*v.x+static_cast<long double>(out.force.y)*v.y+
    static_cast<long double>(out.force.z)*v.z+static_cast<long double>(out.couple.x)*omega.x+
    static_cast<long double>(out.couple.y)*omega.y+static_cast<long double>(out.couple.z)*omega.z;
  EXPECT_NEAR(static_cast<double>(group_power-member_power),0,5e-14);
}
TEST(NodalRigidGroupMath,WrenchCovariesUnderRigidFrameAndOriginChanges) {
  const Vec3 x[]{{1,2,3},{-.5,.4,.9},{.3,-.7,.2}},f[]{{2,3,5},{7,-4,2},{-1,9,3}},c[]{{.1,.2,.3},{-.4,.8,.2},{.3,-.1,.6}};
  const Vec3 center{.25,.1,-.5};
  const auto rotate=[](Vec3 p)->Vec3 { return {p.z,p.x,p.y}; };
  const auto transform=[&](Vec3 p)->Vec3 { const auto r=rotate(p); return {r.x+9,r.y-2,r.z+.5}; };
  Vec3 rx[3],rf[3],rc[3];
  for(unsigned i=0;i<3;++i) { rx[i]=transform(x[i]); rf[i]=rotate(f[i]); rc[i]=rotate(c[i]); }
  rigid::Wrench a,b;
  ASSERT_EQ(rigid::AggregateWrench(center,x,f,c,3,a),rigid::MathStatus::Success);
  ASSERT_EQ(rigid::AggregateWrench(transform(center),rx,rf,rc,3,b),rigid::MathStatus::Success);
  Near(b.force,rotate(a.force)); Near(b.couple,rotate(a.couple));
}
TEST(NodalRigidGroupMath,AnisotropicGyroAccelerationBalancesWorldAngularMomentumEquation) {
  const auto frame=Frame(); const Vec3 w{.7,-1.2,.4},torque{2,-3,1}; Vec3 acceleration;
  ASSERT_EQ(rigid::AngularAcceleration(frame,w,torque,acceleration),rigid::MathStatus::Success);
  // This signed permutation makes the independently formed world tensor
  // diagonal(5,2,3), so verify I*alpha + omega cross (I*omega) = applied torque.
  const long double j[]{5,2,3},wv[]{w.x,w.y,w.z};
  long double momentum[3]; for(unsigned a=0;a<3;++a) momentum[a]=j[a]*wv[a];
  const long double gyro[]{wv[1]*momentum[2]-wv[2]*momentum[1],wv[2]*momentum[0]-wv[0]*momentum[2],wv[0]*momentum[1]-wv[1]*momentum[0]};
  for(unsigned a=0;a<3;++a) EXPECT_NEAR(static_cast<double>(j[a]*Get(acceleration,a)+gyro[a]),Get(torque,a),2e-15);
}
TEST(NodalRigidGroupMath,SphericalLimitAndPureCoupleDoNotIntroduceSpuriousGyroTerms) {
  auto frame=Frame(); frame.inertia={4,4,4}; Vec3 result;
  ASSERT_EQ(rigid::AngularAcceleration(frame,{9,-17,35},{4,8,-12},result),rigid::MathStatus::Success);
  Near(result,{1,2,-3},0);
  frame=Frame(); ASSERT_EQ(rigid::AngularAcceleration(frame,{}, {5,4,9},result),rigid::MathStatus::Success);
  Near(result,{1,2,3},0);
}
TEST(NodalRigidGroupMath,LateFailuresAndOverflowPreserveCallerOutputs) {
  Vec3 x[3]{{1,0,0},{0,1,0},{0,0,1}},f[3]{{1,2,3},{4,5,6},{7,8,9}},c[3]{};
  rigid::Wrench output{{11,12,13},{14,15,16}}; const auto saved=output;
  f[2].z=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(rigid::AggregateWrench({},x,f,c,3,output),rigid::MathStatus::InvalidInput);
  Near(output.force,saved.force,0); Near(output.couple,saved.couple,0);
  f[2]={0,std::numeric_limits<double>::max(),0}; x[2]={2,0,0};
  EXPECT_EQ(rigid::AggregateWrench({},x,f,c,3,output),rigid::MathStatus::NonfiniteResult);
  Near(output.force,saved.force,0); Near(output.couple,saved.couple,0);
  auto frame=Frame(); frame.axes.v[0]=1;
  Vec3 result{11,12,13};
  EXPECT_EQ(rigid::AngularAcceleration(frame,{1,2,3},{4,5,6},result),rigid::MathStatus::InvalidInput);
  Near(result,{11,12,13},0);
  frame=Frame(); frame.inertia={1,2,3};
  EXPECT_EQ(rigid::AngularAcceleration(frame,{1e308,1e308,1e308},{},result),rigid::MathStatus::NonfiniteResult);
  Near(result,{11,12,13},0);
}
} // namespace rigid_test
