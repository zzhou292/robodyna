#include "Fixture.h"
#include <gtest/gtest.h>
#include <cstring>
#include <limits>
namespace qbat_test {
TEST(QbatGeometry, RectangularReferenceIndependentMassAndCoefficientLedger) {
  auto in=Fixture(false);
  qb::Reference ref;
  ASSERT_EQ(qb::InitializeReference(in,ref),qb::Status::kSuccess);
  const auto& q=ref.quadrilateral();
  const long double area=.04L*.02L,t=in.quadrilateral.thickness,rho=in.quadrilateral.density;
  const long double mass=rho*t*area/4;
  EXPECT_NEAR(q.area,static_cast<double>(area),2e-18);
  for(unsigned i=0;i<4;++i) {
    EXPECT_NEAR(q.nodal_mass[i],static_cast<double>(mass),2e-18);
    EXPECT_NEAR(q.physical_inertia[i],static_cast<double>(mass*t*t/12),2e-25);
    EXPECT_NEAR(q.added_inertia[i],static_cast<double>(mass*area/12),2e-23);
    EXPECT_NEAR(q.isotropic_inertia[i],static_cast<double>(mass*(area+t*t)/12),2e-23);
  }
  long double fac2=static_cast<long double>(3.413f)*(1-static_cast<long double>(.7071f));
  fac2=static_cast<long double>(.78f)+static_cast<long double>(.22f)*fac2*fac2*fac2;
  const long double length=area/std::sqrt(2*1.25L*fac2*(4.L/3)*(.02L*.02L+.01L*.01L));
  const long double sound=std::sqrt(static_cast<long double>(in.quadrilateral.young_modulus)/(1-.3L*.3L)/rho);
  const auto& c=ref.coefficients();
  EXPECT_NEAR(c.characteristic_length_m,static_cast<double>(length),1e-16);
  EXPECT_NEAR(c.unscaled_element_dt_s,static_cast<double>(length/sound),1e-18);
  EXPECT_DOUBLE_EQ(c.viscosity_timestep_factor,1);
  EXPECT_NEAR(c.nodal_translation_stiffness_n_m,
      static_cast<double>(.5L*t*area*in.initial_a11_pa/(length*length)),1e-9);
}
TEST(QbatGeometry, FourPointsAndNativeRoundedGaussConstant) {
  const auto in=Fixture(false);
  qb::Reference ref;
  ASSERT_EQ(qb::InitializeReference(in,ref),qb::Status::kSuccess);
  qb::Geometry g;
  ASSERT_EQ(qb::EvaluateGeometry(ref,Current(in),g),qb::Status::kSuccess);
  const double pg=static_cast<double>(.577350269189626f);
  EXPECT_GT(std::abs(pg-std::sqrt(1./3)),1e-9);
  for(const auto& p:g.point) EXPECT_DOUBLE_EQ(p.jacobian_m2,g.area_m2*.25);
  EXPECT_NEAR(g.point[0].hx_per_m,-2*pg/.04,1e-13);
  EXPECT_NEAR(g.point[0].hy_per_m,-2*pg/.02,1e-13);
  EXPECT_LT(g.point[1].hx_per_m,0);
  EXPECT_GT(g.point[1].hy_per_m,0);
  EXPECT_GT(g.point[2].hy_per_m,0);
  EXPECT_NE(g.point[0].membrane_b_per_m[2],g.point[2].membrane_b_per_m[2]);
  // Native IDRIL0 shear is constant; its coefficients are not a per-point Bxy.
  EXPECT_DOUBLE_EQ(g.assumed_shear_per_m[0],-.5/.04);
  EXPECT_DOUBLE_EQ(g.assumed_shear_per_m[3],-.5/.02);
}
TEST(QbatGeometry, WarpedCurrentGeometryPreservesWarpAndAllPointAreas) {
  const auto in=Fixture();
  qb::Reference ref;
  ASSERT_EQ(qb::InitializeReference(in,ref),qb::Status::kSuccess);
  qb::Geometry g;
  ASSERT_EQ(qb::EvaluateGeometry(ref,Current(in),g),qb::Status::kSuccess);
  EXPECT_GT(std::abs(g.actual_warpage_m),1e-5);
  EXPECT_NE(g.point[0].jacobian_m2,g.point[1].jacobian_m2);
  double sum=0;
  for(const auto& p:g.point) sum+=p.jacobian_m2;
  EXPECT_NEAR(sum,g.area_m2,2e-18);
  auto moved=Current(in);
  for(auto& x:moved.position_m) x=Transform(x,1.2,.7);
  qb::Geometry rotated;
  ASSERT_EQ(qb::EvaluateGeometry(ref,moved,rotated),qb::Status::kSuccess);
  EXPECT_NEAR(rotated.area_m2,g.area_m2,1e-16);
  EXPECT_NEAR(rotated.actual_warpage_m,g.actual_warpage_m,4e-16);
  EXPECT_NE(rotated.frame.v[0],g.frame.v[0]);
  EXPECT_NEAR(rotated.point[3].hx_per_m,g.point[3].hx_per_m,2e-11);
}
TEST(QbatGeometry, ExplicitOptionsAndLateCoefficientFailurePreserveReference) {
  auto in=Fixture();
  qb::Reference ref;
  ASSERT_EQ(qb::InitializeReference(in,ref),qb::Status::kSuccess);
  const auto before=ref;
  in.options.ihbe=23;
  EXPECT_EQ(qb::InitializeReference(in,ref),qb::Status::kInvalidInput);
  EXPECT_EQ(std::memcmp(&ref,&before,sizeof(ref)),0);
  in=before.input();
  in.options.nptt=3;
  EXPECT_EQ(qb::InitializeReference(in,ref),qb::Status::kInvalidInput);
  in=before.input();
  in.options.idrill=1;
  EXPECT_EQ(qb::InitializeReference(in,ref),qb::Status::kInvalidInput);
  in=before.input();
  in.initial_a11_pa=std::numeric_limits<double>::max();
  in.quadrilateral.thickness=1;
  EXPECT_EQ(qb::InitializeReference(in,ref),qb::Status::kNonfiniteResult);
  EXPECT_EQ(std::memcmp(&ref,&before,sizeof(ref)),0);
  // Consuming a borrowed previous input is safe; failure never edits it.
  ASSERT_EQ(qb::InitializeReference(ref.input(),ref),qb::Status::kSuccess);
  EXPECT_EQ(Values(ref),Values(before));
}
TEST(QbatGeometry, GeometryFailureAndExactRetryAreValueAtomic) {
  auto in=Fixture();
  qb::Reference ref;
  ASSERT_EQ(qb::InitializeReference(in,ref),qb::Status::kSuccess);
  auto current=Current(in);
  qb::Geometry g;
  ASSERT_EQ(qb::EvaluateGeometry(ref,current,g),qb::Status::kSuccess);
  const auto before=g;
  current.position_m[3]=current.position_m[1];
  EXPECT_NE(qb::EvaluateGeometry(ref,current,g),qb::Status::kSuccess);
  EXPECT_EQ(std::memcmp(&g,&before,sizeof(g)),0);
  current=Current(in);
  current.native_off=2;
  EXPECT_EQ(qb::EvaluateGeometry(ref,current,g),qb::Status::kInvalidInput);
  current=Current(in);
  current.position_m[3].z=std::numeric_limits<double>::infinity();
  EXPECT_EQ(qb::EvaluateGeometry(ref,current,g),qb::Status::kInvalidInput);
  EXPECT_EQ(std::memcmp(&g,&before,sizeof(g)),0);
  EXPECT_EQ(qb::EvaluateGeometry(qb::Reference{},Current(in),g),qb::Status::kInvalidReference);
  ASSERT_EQ(qb::EvaluateGeometry(ref,Current(in),g),qb::Status::kSuccess);
  EXPECT_EQ(Values(g),Values(before));
}
TEST(QbatGeometry, ResolvedViscositiesStaySeparateFromForceDefaults) {
  auto in=Fixture();
  qb::Reference zero,damped;
  ASSERT_EQ(qb::InitializeReference(in,zero),qb::Status::kSuccess);
  in.options.membrane_viscosity=.035;
  in.options.numerical_viscosity=.002;
  ASSERT_EQ(qb::InitializeReference(in,damped),qb::Status::kSuccess);
  EXPECT_LT(damped.coefficients().unscaled_element_dt_s,zero.coefficients().unscaled_element_dt_s);
  EXPECT_GT(damped.coefficients().nodal_rotation_stiffness_nm,zero.coefficients().nodal_rotation_stiffness_nm);
  EXPECT_EQ(damped.quadrilateral().nodal_mass[0],zero.quadrilateral().nodal_mass[0]);
  in.options.numerical_viscosity=std::numeric_limits<double>::max();
  const auto old=Values(damped);
  EXPECT_EQ(qb::InitializeReference(in,damped),qb::Status::kNonfiniteResult);
  EXPECT_EQ(Values(damped),old);
}
} // namespace qbat_test
