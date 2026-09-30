#include "Fixture.h"
#include <limits>

namespace rigid_contact_test {
TEST(RigidContactResponse, CenterAndOffsetUseActualAnisotropicBodyMass) {
  const auto body=Body();sc::RigidNormalResponse response;
  ASSERT_EQ(sc::EvaluateRigidNormalResponse(body,body.center,{-1,0,0},response),sc::Status::kOk);
  EXPECT_DOUBLE_EQ(response.inverse_effective_mass,.5);
  for(const Vec3 x : {Vec3{.4,2,-3},Vec3{1e6,1e6+1,1e6-1},Vec3{-.25,.5,2}}) {
    ASSERT_EQ(sc::EvaluateRigidNormalResponse(body,x,{-1,0,0},response),sc::Status::kOk);
    const auto expected=Oracle(body,x,{-1,0,0});
    EXPECT_NEAR(response.inverse_effective_mass,static_cast<double>(expected),
                static_cast<double>(expected)*1e-14);
    EXPECT_GE(static_cast<long double>(response.inverse_upper),expected);
  }
}
TEST(RigidContactResponse, NormalReversalAndRigidWorldRotationPreserveResponse) {
  auto body=Body();const Vec3 point{.4,2,-3},normal{.6,.8,0};sc::RigidNormalResponse base,reversed,turned;
  ASSERT_EQ(sc::EvaluateRigidNormalResponse(body,point,normal,base),sc::Status::kOk);
  ASSERT_EQ(sc::EvaluateRigidNormalResponse(body,point,{-.6,-.8,0},reversed),sc::Status::kOk);
  EXPECT_DOUBLE_EQ(base.inverse_effective_mass,reversed.inverse_effective_mass);
  const auto turn=[](Vec3 v){return Vec3{-v.y,v.x,v.z};};
  body.center=turn(body.center);
  for(unsigned axis=0;axis<3;++axis) {
    const auto v=turn(rigid::detail::Column(body.current_frame.axes,axis));
    body.current_frame.axes.v[axis]=v.x;body.current_frame.axes.v[3+axis]=v.y;body.current_frame.axes.v[6+axis]=v.z;
  }
  ASSERT_EQ(sc::EvaluateRigidNormalResponse(body,turn(point),turn(normal),turned),sc::Status::kOk);
  EXPECT_NEAR(turned.inverse_effective_mass,base.inverse_effective_mass,1e-14);
  EXPECT_GE(static_cast<long double>(turned.inverse_upper),Oracle(body,turn(point),turn(normal)));
}
TEST(RigidContactResponse, InvalidFinalInertiaAndOverflowPreservePriorValues) {
  auto body=Body();sc::RigidNormalResponse response{17,18,true};const auto before=response;
  body.current_frame.inertia.z=0;
  EXPECT_EQ(sc::EvaluateRigidNormalResponse(body,{1,2,3},{-1,0,0},response),sc::Status::kInvalidArgument);
  body=Body();body.current_frame.axes.v[8]=2;
  EXPECT_EQ(sc::EvaluateRigidNormalResponse(body,{1,2,3},{-1,0,0},response),sc::Status::kInvalidArgument);
  body=Body();
  EXPECT_EQ(sc::EvaluateRigidNormalResponse(body,{1,2,3},{0,0,0},response),sc::Status::kInvalidArgument);
  EXPECT_EQ(sc::EvaluateRigidNormalResponse(body,{1,std::numeric_limits<double>::max(),3},{-1,0,0},response),
            sc::Status::kNonFiniteResult);
  EXPECT_EQ(response.inverse_effective_mass,before.inverse_effective_mass);
  EXPECT_EQ(response.inverse_upper,before.inverse_upper);EXPECT_EQ(response.valid,before.valid);
  double trace=3;
  EXPECT_EQ(sc::AccumulateRigidContactTrace(-1,response,trace),sc::Status::kInvalidArgument);
  EXPECT_EQ(trace,3);
  EXPECT_EQ(sc::AccumulateRigidContactTrace(std::numeric_limits<double>::max(),response,trace),
            sc::Status::kNonFiniteResult);EXPECT_EQ(trace,3);
}
} // namespace rigid_contact_test
