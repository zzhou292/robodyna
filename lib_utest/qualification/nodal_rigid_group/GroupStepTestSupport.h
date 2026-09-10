#pragma once
#include "GroupTestSupport.h"
#include "lib_src/constraints/NodalRigidGroupStepMath.h"
#include <algorithm>
#include <cstring>

#if defined(__CUDACC__)
#define RIGID_TEST_HD __host__ __device__
#else
#define RIGID_TEST_HD
#endif
namespace rigid_step_test {
using namespace rigid_test;
inline constexpr unsigned Count=4;
struct Input { rigid::PrimaryStepInput body; rigid::MemberStepInput member[Count]; };
struct Trial { rigid::PrimaryStepTrial primary; rigid::MemberStepTrial member[Count]; };
inline rigid::PrincipalFrame DenseFrame() {
  // Rotation from unit quaternion (1,2,3,4)/sqrt(30), written independently.
  return {{{-2./3,2./15,11./15,2./3,-1./3,2./3,1./3,14./15,2./15}},{1.044,.804,.404}};
}
inline Input Fixture(rigid::StepDurations durations={1./128,1./128,1./128}) {
  Input input; auto& b=input.body; b.previous_frame=DenseFrame(); b.center={.2,-.1,.3};
  b.velocity={.3,-.2,.1}; b.omega={.7,-1.2,.4}; b.mass=8; b.durations=durations;
  const Vec3 local[]{{-.1,-.2,-.3},{-.1,.2,.3},{.1,-.2,.3},{.1,.2,-.3}};
  const Vec3 forces[]{{2,3,5},{7,-4,2},{-1,9,3},{-3,2,-1}};
  const Vec3 couples[]{{.1,.2,.3},{-.4,.8,.2},{.3,-.1,.6},{.7,-.2,.1}};
  for(unsigned i=0;i<Count;++i) {
    const auto r=rigid::detail::ToWorld(b.previous_frame.axes,local[i]);
    auto& m=input.member[i]; m.position=rigid::detail::Add(b.center,r);
    m.velocity=rigid::detail::Add(b.velocity,rigid::detail::Cross(b.omega,r));
    m.omega=b.omega; m.mass=2; m.inertia=.001; m.force=forces[i]; m.couple=couples[i];
  }
  return input;
}
RIGID_TEST_HD inline rigid::StepStatus EvaluatePacket(const Input& input,Trial& output) {
  Vec3 positions[Count],forces[Count],couples[Count];
  for(unsigned i=0;i<Count;++i) { positions[i]=input.member[i].position;
    forces[i]=input.member[i].force; couples[i]=input.member[i].couple; }
  auto body=input.body;
  const auto wrench=rigid::AggregateWrench(body.center,positions,forces,couples,Count,body.applied);
  if(wrench!=rigid::MathStatus::Success)
    return wrench==rigid::MathStatus::InvalidInput?rigid::StepStatus::InvalidInput:rigid::StepStatus::NonfiniteResult;
  Trial next; auto status=rigid::EvaluatePrimaryStep(body,next.primary);
  if(status!=rigid::StepStatus::Success) return status;
  for(unsigned i=0;i<Count;++i) {
    status=rigid::EvaluateMemberStep(body,next.primary,input.member[i],next.member[i]);
    if(status!=rigid::StepStatus::Success) return status;
  }
  output=next; return rigid::StepStatus::Success;
}
inline void Agreement(double actual,double expected,double relative=2e-12) {
  ASSERT_TRUE(std::isfinite(actual)); ASSERT_TRUE(std::isfinite(expected));
  EXPECT_LE(std::abs(actual-expected),relative*std::max({1.,std::abs(actual),std::abs(expected)}));
}
inline void Agreement(Vec3 a,Vec3 b,double tolerance=2e-12) {
  for(unsigned i=0;i<3;++i) Agreement(Get(a,i),Get(b,i),tolerance);
}
inline void Agreement(const Trial& a,const Trial& b,double tolerance=2e-12) {
  for(unsigned i=0;i<9;++i) Agreement(a.primary.force_frame.axes.v[i],b.primary.force_frame.axes.v[i],tolerance);
  Agreement(a.primary.force_frame.inertia,b.primary.force_frame.inertia,tolerance);
  Agreement(a.primary.saved_body_omega,b.primary.saved_body_omega,tolerance);
  Agreement(a.primary.acceleration,b.primary.acceleration,tolerance);
  Agreement(a.primary.angular_acceleration,b.primary.angular_acceleration,tolerance);
  Agreement(a.primary.center,b.primary.center,tolerance); Agreement(a.primary.velocity,b.primary.velocity,tolerance);
  Agreement(a.primary.omega,b.primary.omega,tolerance);
  for(unsigned i=0;i<Count;++i) {
    const auto& x=a.member[i]; const auto& y=b.member[i];
    Agreement(x.acceleration,y.acceleration,tolerance); Agreement(x.angular_acceleration,y.angular_acceleration,tolerance);
    Agreement(x.position,y.position,tolerance); Agreement(x.velocity,y.velocity,tolerance); Agreement(x.omega,y.omega,tolerance);
    Agreement(x.reaction_force,y.reaction_force,tolerance); Agreement(x.reaction_couple,y.reaction_couple,tolerance);
  }
}
template<class T> std::array<unsigned char,sizeof(T)> Bytes(const T& value) {
  std::array<unsigned char,sizeof(T)> bytes; std::memcpy(bytes.data(),&value,sizeof value); return bytes;
}
} // namespace rigid_step_test
#undef RIGID_TEST_HD
