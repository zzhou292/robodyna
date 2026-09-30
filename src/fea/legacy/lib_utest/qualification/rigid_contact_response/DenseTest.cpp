#include "Fixture.h"
#include <Eigen/Eigenvalues>

namespace rigid_contact_test {
TEST(RigidContactResponse, CompleteBodyTraceBoundsIndependentSixDofContactEigenvalues) {
  const auto body=Body();Eigen::Matrix<double,6,6> operator_matrix=Eigen::Matrix<double,6,6>::Zero();
  double trace=0;long double exact_trace=0;
  const Vec3 positions[]{{-.4,2,1},{1,.2,-.7},{2,-3,.5},{-.3,.1,2}};
  const Vec3 normals[]{{1,0,0},{0,1,0},{0,0,1},{.6,.8,0}};
  for(unsigned row=0;row<4;++row) {
    const auto p=positions[row],n=normals[row];const double stiffness=1200*(row+1);
    sc::RigidNormalResponse response;
    ASSERT_EQ(sc::EvaluateRigidNormalResponse(body,p,n,response),sc::Status::kOk);
    ASSERT_EQ(sc::AccumulateRigidContactTrace(stiffness,response,trace),sc::Status::kOk);
    exact_trace+=stiffness*Oracle(body,p,n);
    // Independent assembled mass-normalized six-DOF operator. No production
    // cross/rotation helpers or response factors are used by this oracle.
    Eigen::Vector3d r(p.x-body.center.x,p.y-body.center.y,p.z-body.center.z),normal(n.x,n.y,n.z);
    Eigen::Matrix3d axes;
    for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)axes(i,j)=body.current_frame.axes.v[3*i+j];
    const Eigen::Vector3d rotational=axes.transpose()*r.cross(normal);
    Eigen::Matrix<double,6,1> jacobian;
    jacobian.head<3>()=normal/std::sqrt(body.mass);
    jacobian.tail<3>()=rotational.cwiseQuotient(Eigen::Vector3d(
        std::sqrt(body.current_frame.inertia.x),std::sqrt(body.current_frame.inertia.y),std::sqrt(body.current_frame.inertia.z)));
    operator_matrix+=stiffness*jacobian*jacobian.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>> eigen(operator_matrix);
  ASSERT_EQ(eigen.info(),Eigen::Success);
  EXPECT_GE(static_cast<long double>(trace),exact_trace);
  EXPECT_LE(eigen.eigenvalues().maxCoeff(),trace);
  EXPECT_NEAR(operator_matrix.trace(),static_cast<double>(exact_trace),static_cast<double>(exact_trace)*1e-14);
}
} // namespace rigid_contact_test
