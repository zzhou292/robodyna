// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
namespace distortion_test {
TEST(DistortionParameters, MechanicalSlotsMatchAuthenticatedReaderAndUpdate) {
  for(const auto& value:Cases()) {
    m::MechanicalSlots slots;ASSERT_EQ(m::PrepareMechanicalSlots(value.material,slots),m::Status::Ok);
    const std::array<double,6> actual{slots.pm20_young_pa,slots.pm21_poisson_ratio,slots.pm22_gs_pa,
      slots.pm32_pa,slots.pm100_reader_bulk_pa,slots.pm107_control_pa};
    const auto expected=NativeSlots(value);
    for(unsigned i=0;i<6;++i)EXPECT_DOUBLE_EQ(actual[i],expected[i]);
    EXPECT_DOUBLE_EQ(slots.pm32_pa,slots.pm22_gs_pa);
  }
}
TEST(DistortionParameters, NativeParametersMatchAcrossMaterialAndKinematicInputs) {
  for(const auto& value:Cases())Check(value);
}
TEST(DistortionParameters, WidenedDefaultRealThresholdAndUpperAdmission) {
  const double boundary=static_cast<double>(.4f);
  auto at=Base(boundary),above=Base(std::nextafter(boundary,1.0));
  EXPECT_DOUBLE_EQ(Check(at).damping_coefficient,5.0/100.0);
  EXPECT_LT(Check(above).damping_coefficient,5.0/100.0);
  Check(Base(d::MaximumPoissonRatio));
  auto unsupported=Base(std::nextafter(d::MaximumPoissonRatio,1.0));auto result=Sentinel();
  EXPECT_EQ(d::PrepareParameters(unsupported.material,unsupported.input,result),d::Status::UnsupportedProfile);
  Same(result,Sentinel());
}
TEST(DistortionParameters, FullHydrostaticStressSetsDampingWithoutDeviatoricProjection) {
  auto value=Base();const auto zero=Check(value);
  for(unsigned i=0;i<3;++i)value.input.cauchy_stress_pa[i]=NativeC1(value)*.02;
  const auto hydrostatic=Check(value);
  EXPECT_GT(hydrostatic.damping_n_s_m2,zero.damping_n_s_m2*900);
  EXPECT_EQ(hydrostatic.buckling_flag,0);
  EXPECT_DOUBLE_EQ(hydrostatic.control_stiffness_n_m,zero.control_stiffness_n_m);
}
TEST(DistortionParameters, ComponentMinimumIsStrictAndNotPrincipalStress) {
  auto value=Base();const double c1=NativeC1(value);
  for(unsigned component=0;component<6;++component) {
    for(double& x:value.input.cauchy_stress_pa)x=0;
    value.input.cauchy_stress_pa[component]=-c1;
    EXPECT_EQ(Check(value).buckling_flag,0);
    value.input.cauchy_stress_pa[component]=std::nextafter(-c1,-std::numeric_limits<double>::infinity());
    EXPECT_EQ(Check(value).buckling_flag,1);
  }
  for(double& x:value.input.cauchy_stress_pa)x=0;
  value.input.cauchy_stress_pa[3]=2*c1;
  // Positive pure shear has a negative principal stress, but native SCRE_SIG3
  // sees no negative stored component and must leave ISTAB at zero.
  EXPECT_EQ(Check(value).buckling_flag,0);
  value.input.cauchy_stress_pa[3]=-2*c1;EXPECT_EQ(Check(value).buckling_flag,1);
}
TEST(DistortionParameters, StressDampingFloorCapAndResponseScaling) {
  auto value=Base();const auto zero=Check(value);const double c1=NativeC1(value);
  value.input.cauchy_stress_pa[3]=c1*1e-8;EXPECT_DOUBLE_EQ(Check(value).damping_n_s_m2,zero.damping_n_s_m2);
  value.input.cauchy_stress_pa[3]=c1;const auto cap=Check(value);
  value.input.cauchy_stress_pa[3]=100*c1;EXPECT_DOUBLE_EQ(Check(value).damping_n_s_m2,cap.damping_n_s_m2);
  value.input.density_kg_m3*=2;value.input.material_sound_speed_m_s*=3;
  const auto scaled=Check(value);EXPECT_NEAR(scaled.damping_n_s_m2,6*cap.damping_n_s_m2,1e-10);
  EXPECT_DOUBLE_EQ(scaled.control_stiffness_n_m,cap.control_stiffness_n_m);
}
TEST(DistortionParameters, InvalidProfileMaterialAndLateOverflowDoNotPublish) {
  const auto valid=Base();
  auto reject=[&](const Case& value,d::Status status){auto result=Sentinel();
    EXPECT_EQ(d::PrepareParameters(value.material,value.input,result),status);Same(result,Sentinel());};
  auto value=valid;value.material.bulk_pa=std::nextafter(value.material.bulk_pa,0.0);
  reject(value,d::Status::InvalidMaterial);
  m::MechanicalSlots slots{1,2,3,4,5,6};const auto unchanged=slots;
  EXPECT_EQ(m::PrepareMechanicalSlots(value.material,slots),m::Status::InvalidParameters);
  EXPECT_DOUBLE_EQ(slots.pm100_reader_bulk_pa,unchanged.pm100_reader_bulk_pa);
  EXPECT_DOUBLE_EQ(slots.pm107_control_pa,unchanged.pm107_control_pa);
  for(unsigned field=0;field<4;++field) {value=valid;
    if(field==0)value.input.off=0;if(field==1)value.input.offg=0;
    if(field==2)value.input.ismstr=12;if(field==3)value.input.units=static_cast<d::WorkingUnits>(2);
    reject(value,d::Status::UnsupportedProfile);}
  value=valid;value.input.current_volume_m3=0;reject(value,d::Status::InvalidInput);
  value=valid;value.input.material_sound_speed_m_s=-1;reject(value,d::Status::InvalidInput);
  value=valid;value.input.cauchy_stress_pa[5]=std::numeric_limits<double>::quiet_NaN();reject(value,d::Status::InvalidInput);
  value=Base(0);ASSERT_EQ(m::Prepare(1e307,0,1100,1e12,value.material),m::Status::Ok);
  value.input.current_volume_m3=1e300;reject(value,d::Status::NonfiniteResult);
  Check(valid);
}
} // namespace distortion_test
