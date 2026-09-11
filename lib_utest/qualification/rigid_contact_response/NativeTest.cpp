#include "Fixture.h"
extern "C" {
void nodal_rigid_native_wrench(const int*,const double*,const double*,const double*,const double*,double*);
void nodal_rigid_native_gyro(const double*,const double*,const double*,double*);
void nodal_rigid_native_frame(const double*,const double*,const double*,double*);
}
namespace rigid_contact_test {
TEST(RigidContactResponseNative, AcceptedFrameAdvancesToTheNativeCurrentForceFrame) {
  const auto body=Body();tl::fea::NodalRigidGroupState accepted;
  accepted.center=body.center;accepted.principal_axes=body.current_frame.axes;accepted.omega={.2,-.3,.4};
  const double local_spin[]{-.3,.4,.2}; // Independent exact cyclic basis transform.
  double previous[9];
  for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)previous[3*j+i]=accepted.principal_axes.v[3*i+j];
  for(double h : {0.,1e-8,.01}) {
    double native[9];nodal_rigid_native_frame(previous,local_spin,&h,native);
    sc::RigidContactBody current;
    ASSERT_EQ(sc::PrepareRigidContactBodyFromAccepted(accepted,body.mass,body.current_frame.inertia,h,current),sc::Status::kOk);
    for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)
      EXPECT_NEAR(current.current_frame.axes.v[3*i+j],native[3*j+i],2e-15);
    EXPECT_EQ(current.center.x,accepted.center.x);EXPECT_EQ(current.mass,body.mass);
    sc::RigidNormalResponse response;
    ASSERT_EQ(sc::EvaluateRigidNormalResponse(current,{.4,2,-3},{-1,0,0},response),sc::Status::kOk);
    EXPECT_GE(static_cast<long double>(response.inverse_upper),Oracle(current,{.4,2,-3},{-1,0,0}));
  }
}
TEST(RigidContactResponseNative, UnitPointForceAgreesWithPinnedWrenchAndPrincipalAcceleration) {
  const auto body=Body();const int count=1;
  const double point[]{.4,2,-3},center[]{body.center.x,body.center.y,body.center.z},
      force[]{.6,.8,0},zero[]{0,0,0},inertia[]{3,5,7};
  double wrench[6];nodal_rigid_native_wrench(&count,point,center,force,zero,wrench);
  // Body() uses an exact cyclic world-to-body permutation.
  const double local_couple[]{wrench[4],wrench[5],wrench[3]};double local_alpha[3];
  nodal_rigid_native_gyro(inertia,zero,local_couple,local_alpha);
  const double alpha[]{local_alpha[2],local_alpha[0],local_alpha[1]},
      r[]{point[0]-center[0],point[1]-center[1],point[2]-center[2]};
  const double acceleration[]{force[0]/body.mass+alpha[1]*r[2]-alpha[2]*r[1],
      force[1]/body.mass+alpha[2]*r[0]-alpha[0]*r[2],force[2]/body.mass+alpha[0]*r[1]-alpha[1]*r[0]};
  const double expected=force[0]*acceleration[0]+force[1]*acceleration[1]+force[2]*acceleration[2];
  sc::RigidNormalResponse result;
  ASSERT_EQ(sc::EvaluateRigidNormalResponse(body,{point[0],point[1],point[2]},
      {force[0],force[1],force[2]},result),sc::Status::kOk);
  EXPECT_NEAR(result.inverse_effective_mass,expected,expected*1e-14);
  EXPECT_GE(result.inverse_upper,expected);
}
} // namespace rigid_contact_test
