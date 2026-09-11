// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>
namespace cin_step_test {
using M6=Eigen::MatrixXd;
TEST(CinPhysicalStepMatrix, FullCoupledRigidSurrogateEigenvaluesAreBoundedWithLeverAndTorqueControls) {
  Fixture f;
  auto prior=fe::rigid::ReadGroupState(f.accepted.data()+19*Fixture::N+18);
  prior.omega={.3,-.2,.1};
  tlfea::contact::RigidContactBody body;
  ASSERT_EQ(tlfea::contact::PrepareRigidContactBodyFromAccepted(prior,3,{.25,.5,1},.2,body),
      tlfea::contact::Status::kOk);
  Eigen::Matrix3d axes;
  for(int r=0;r<3;++r) for(int c=0;c<3;++c) axes(r,c)=body.current_frame.axes.v[3*r+c];
  M6 k=M6::Zero(6,6), inverse_sqrt=M6::Zero(6,6);
  inverse_sqrt.topLeftCorner<3,3>()=Eigen::Matrix3d::Identity()/std::sqrt(body.mass);
  inverse_sqrt.bottomRightCorner<3,3>()=axes*Eigen::Vector3d(2,std::sqrt(2.),1).asDiagonal()*axes.transpose();
  double trace=0, without_lever=0, without_rotation=0;
  for(unsigned node: {4u,5u}) {
    const auto* p=f.accepted.data()+3*node;
    const Eigen::Vector3d arm(p[0]-body.center.x,p[1]-body.center.y,p[2]-body.center.z);
    Eigen::Matrix3d cross;
    cross<<0,-arm.z(),arm.y(),arm.z(),0,-arm.x(),-arm.y(),arm.x(),0;
    M6 b=M6::Zero(6,6), diagonal=M6::Zero(6,6);
    b.topLeftCorner<3,3>().setIdentity(); b.topRightCorner<3,3>()=-cross;
    b.bottomRightCorner<3,3>().setIdentity();
    diagonal.topLeftCorner<3,3>()=f.translation[node]*Eigen::Matrix3d::Identity();
    diagonal.bottomRightCorner<3,3>()=f.rotation[node]*Eigen::Matrix3d::Identity();
    k+=b.transpose()*diagonal*b;
    ASSERT_TRUE(dt::AddRigidMemberTrace(body,{p[0],p[1],p[2]},f.translation[node],f.rotation[node],trace));
    ASSERT_TRUE(dt::AddRigidMemberTrace(body,body.center,f.translation[node],f.rotation[node],without_lever));
    ASSERT_TRUE(dt::AddRigidMemberTrace(body,{p[0],p[1],p[2]},f.translation[node],0,without_rotation));
  }
  const M6 normalized=inverse_sqrt*k*inverse_sqrt;
  const Eigen::SelfAdjointEigenSolver<M6> eigen(normalized);
  ASSERT_EQ(eigen.info(),Eigen::Success);
  EXPECT_GE(eigen.eigenvalues().minCoeff(),-1e-10);
  EXPECT_GE(trace,eigen.eigenvalues().maxCoeff());
  EXPECT_NEAR(trace,normalized.trace(),normalized.trace()*2e-13);
  EXPECT_LT(without_lever,trace*.75);
  EXPECT_LT(without_rotation,trace-100);
  dt::ScalarLimit limit;
  ASSERT_TRUE(dt::RigidTraceLimit(trace,.8,limit));
  EXPECT_LE(limit.dt,.8*std::sqrt(2/eigen.eigenvalues().maxCoeff()));
  EXPECT_LT(limit.dt,2/std::sqrt(eigen.eigenvalues().maxCoeff()));
  // Joint/contact-independent local test: adding PSD torque stiffness must
  // tighten, never enlarge, the body step despite a zero-mass member.
  double more=trace;
  ASSERT_TRUE(dt::AddRigidMemberTrace(body,body.center,0,100,more));
  dt::ScalarLimit tighter;
  ASSERT_TRUE(dt::RigidTraceLimit(more,.8,tighter));
  EXPECT_LT(tighter.dt,limit.dt);
}
} // namespace cin_step_test
