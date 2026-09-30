#include "lib_src/collision/Q4SurfaceMass.h"
#include "lib_utest/q4_prescribed_contact_fixture.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace {
namespace sc=tlfea::contact;
namespace fixture=q4_prescribed_test;

TEST(Q4UnrestrictedMass, UsesActualFreeXYZMassAndConservativeNormalStencil) {
  q4_planar_test::Single surface; fixture::PhysicalMass<4> mass;
  mass.inverse={.25,1,4,16};
  for (unsigned fixed=0;fixed<2;++fixed) {
    SCOPED_TRACE(fixed);
    if (fixed) { mass.fixed[2]=1; mass.inverse[2]=0; }
    sc::NormalJacobian result;
    ASSERT_EQ(sc::BuildQ4NormalXJacobian(mass.view(),surface.parent,.5,-.25,17,&result),sc::Status::kOk);
    double shape[4]; ASSERT_EQ(sc::EvaluateQ4Shape(.5,-.25,shape),sc::Status::kOk);
    double expected=0;
    for (unsigned local=0;local<4;++local) expected+=shape[local]*shape[local]*mass.inverse[surface.parent.nodes[local]];
    EXPECT_EQ(result.inverse_effective_mass,expected);
    ASSERT_EQ(result.count,4u); EXPECT_TRUE(result.valid); EXPECT_EQ(result.base_epoch,9u); EXPECT_EQ(result.attempt,17u);
    for (unsigned i=0;i<4;++i) {
      EXPECT_EQ(result.nodes[i],i);
      unsigned local=0; while (surface.parent.nodes[local] != i) ++local;
      EXPECT_EQ(result.values[i].x,-shape[local]); EXPECT_EQ(result.values[i].y,0); EXPECT_EQ(result.values[i].z,0);
      EXPECT_GE(result.normalized_norm[i],shape[local]*std::sqrt(mass.inverse[i]));
      if (mass.fixed[i]) { EXPECT_EQ(result.normalized_norm[i],0); }
    }
  }
}

TEST(Q4UnrestrictedMass, TransposeMatchesNormalPowerWithNonuniformTangentialVelocity) {
  q4_planar_test::Single surface; fixture::PhysicalMass<4> mass;
  for (unsigned n=0;n<4;++n) {
    surface.velocity[3*n]=.25+n; surface.velocity[3*n+1]=-3.+n; surface.velocity[3*n+2]=7.-2*n;
  }
  sc::NormalJacobian jacobian; sc::Q4PointKinematics point; sc::Q4NodalForces forces;
  const sc::Q4Point location{0,-.5,.25};
  ASSERT_EQ(sc::BuildQ4NormalXJacobian(mass.view(),surface.parent,location.u,location.v,7,&jacobian),sc::Status::kOk);
  ASSERT_EQ(sc::EvaluateQ4Point(surface.Input().surface,location,&point),sc::Status::kOk);
  ASSERT_EQ(sc::ProjectQ4PointForce(surface.Input().surface,location,{-3.5,0,0},&forces),sc::Status::kOk);
  double normal_speed=0,power=0;
  for (unsigned i=0;i<jacobian.count;++i)
    normal_speed+=sc::Dot(jacobian.values[i],surface.Input().surface.velocities.at(jacobian.nodes[i]));
  for (unsigned i=0;i<4;++i) {
    power+=sc::Dot(forces.forces[i],surface.Input().surface.velocities.at(forces.nodes[i]));
    EXPECT_EQ(sc::Dot(forces.couples[i],forces.couples[i]),0);
  }
  EXPECT_EQ(normal_speed,-point.velocity.x); EXPECT_EQ(power,3.5*normal_speed);
}

TEST(Q4UnrestrictedMass, UnsupportedOrMalformedActualMassPreservesOutputEvenAtZeroWeightNodes) {
  q4_planar_test::Single surface;
  sc::NormalJacobian result; result.valid=true; result.count=3; result.inverse_effective_mass=19; result.attempt=97;
  const auto before=fixture::Bytes(result);
  for (unsigned fault=0;fault<8;++fault) {
    SCOPED_TRACE(fault);
    fixture::PhysicalMass<4> mass; auto view=mass.view();
    if (fault == 0) view.model=sc::TranslationMassModel::kUnspecified;
    if (fault == 1) view.model=sc::TranslationMassModel::kGeneralizedOrRotational;
    if (fault == 2) mass.fixed[0]=6;  // Component bits are not boolean fixed flags.
    if (fault == 3) mass.inverse[0]=0;
    if (fault == 4) { mass.fixed[0]=1; mass.inverse[0]=1; }
    if (fault == 5) mass.inverse[0]=std::numeric_limits<double>::quiet_NaN();
    if (fault == 6) view.inverse_mass=nullptr;
    if (fault == 7) view.fixed=nullptr;
    // Natural node0 is physical node3. Node0 above has ZERO weight here.
    EXPECT_NE(sc::BuildQ4NormalXJacobian(view,surface.parent,1,1,7,&result),sc::Status::kOk);
    EXPECT_EQ(fixture::Bytes(result),before);
  }
  fixture::PhysicalMass<4> fixed; fixed.fixed.fill(1); fixed.inverse.fill(0);
  EXPECT_EQ(sc::BuildQ4NormalXJacobian(fixed.view(),surface.parent,0,0,7,&result),sc::Status::kNoDynamicDofs);
  EXPECT_EQ(fixture::Bytes(result),before);
  fixture::PhysicalMass<4> good;
  EXPECT_EQ(sc::BuildQ4NormalXJacobian(good.view(),surface.parent,0,0,0,&result),sc::Status::kInvalidArgument);
  EXPECT_EQ(fixture::Bytes(result),before);
  EXPECT_EQ(sc::BuildQ4NormalXJacobian(good.view(),surface.parent,0,0,7,&result),sc::Status::kOk);
}

TEST(Q4UnrestrictedMass, CenterMassDoesNotHideLaterGaussUnderflow) {
  q4_planar_test::Single surface; fixture::PhysicalMass<4> mass;
  mass.inverse.fill(16*std::numeric_limits<double>::denorm_min());
  sc::NormalJacobian result;
  ASSERT_EQ(sc::BuildQ4NormalXJacobian(mass.view(),surface.parent,0,0,7,&result),sc::Status::kOk);
  const auto before=fixture::Bytes(result); const double gauss=1/std::sqrt(3.);
  EXPECT_EQ(sc::BuildQ4NormalXJacobian(mass.view(),surface.parent,-gauss,-gauss,7,&result),sc::Status::kNonFiniteResult);
  EXPECT_EQ(fixture::Bytes(result),before);
}
}  // namespace
