#include "Law1TestSupport.h"
#include "../native/law1/NativeReference.h"
namespace layered_law1_test {
namespace native=tl::qualification::law1;
TEST(LayeredLaw1Native, ExactNativeCoefficientPointAndPhysicalThickness) {
  for(const auto tuple:{std::array<double,3>{200e9,.3,7890.},std::array<double,3>{10e9,.3,789.},
                       std::array<double,3>{200e9,0.,7890.}}) {
    const auto p=Material(tuple[0],tuple[1],tuple[2]);PointHistory h{{1e6,-2e6,3e6,-4e6,5e6}};
    PointInput in{{.003,-.001,.0007,-.0005,.0002},p.elastic.g*5./6.,.0005,.002};
    native::PointInput ni;ni.young=p.young_pa;ni.nu=p.poisson_ratio;ni.rho=p.density_kg_m3;
    ni.gs=in.transverse_shear_modulus;ni.layer_thickness=in.layer_thickness;ni.reported_thickness=in.reported_thickness;
    std::copy_n(h.stress,5,ni.stress.begin());std::copy_n(in.strain_increment,5,ni.increment.begin());
    native::PointResult n;PointResult value;
    ASSERT_TRUE(native::Evaluate(ni,&n));ASSERT_TRUE(mat::UpdateShellElasticLaw1Point(p,h,in,value));
    const double coefficients[]{p.elastic.g,p.elastic.a11,p.elastic.a12,p.elastic.sound_speed};
    for(unsigned c=0;c<4;++c)EXPECT_DOUBLE_EQ(coefficients[c],n.coefficients[c]);
    for(unsigned c=0;c<5;++c)EXPECT_DOUBLE_EQ(value.history.stress[c],n.stress[c]);
    EXPECT_DOUBLE_EQ(value.reported_thickness,n.thickness);
  }
}
TEST(LayeredLaw1Native, IndependentLoadedHeldReversedHistoriesAndThicknessPhaseControls) {
  for(double e:{200e9,10e9}) {
    const auto p=Material(e);History history;native::SectionInput native_input;
    native_input.young=e;native_input.nu=p.poisson_ratio;native_input.rho=p.density_kg_m3;
    native_input.gs=p.elastic.g*5./6.;double thickness=.002,native_thickness=.002;
    double excursion=0,reset_difference=0,frozen_difference=0;
    for(unsigned i=0;i<160;++i) {
      const auto in=Step(i,thickness,native_input.gs);Result result;native::SectionResult n;
      native_input.reference_thickness=native_input.reported_thickness=native_thickness;
      std::copy_n(in.strain_curvature_increment,8,native_input.increment.begin());
      ASSERT_TRUE(native::Evaluate(native_input,&n))<<i;
      ASSERT_TRUE(sec::UpdateShellLayeredLaw1(p,history,in,result))<<i;
      for(unsigned k=0;k<3;++k)for(unsigned c=0;c<5;++c)NativeClose(result.history.point[k].stress[c],n.stress[5*k+c]);
      for(unsigned c=0;c<5;++c)NativeClose(result.material_stress[c],n.force[c]);
      for(unsigned c=0;c<3;++c)NativeClose(result.bending_stress[c],n.moment[c]);
      ThicknessClose(result.reported_thickness,n.thickness);
      auto frozen=native_input;frozen.reference_thickness=.002;native::SectionResult control;
      ASSERT_TRUE(native::Evaluate(frozen,&control));
      frozen_difference=std::max(frozen_difference,std::abs(n.moment[0]-control.moment[0]));
      auto reset=native_input;reset.stress={};ASSERT_TRUE(native::Evaluate(reset,&control));
      reset_difference=std::max(reset_difference,std::abs(n.force[0]-control.force[0]));
      excursion=std::max(excursion,std::abs(n.thickness-.002));
      history=result.history;thickness=result.reported_thickness;
      native_input.stress=n.stress;native_thickness=n.thickness;
    }
    EXPECT_GT(excursion,1.e-6);EXPECT_GT(reset_difference,1.e6);EXPECT_GT(frozen_difference,.1);
  }
}
TEST(LayeredLaw1Native, NativeAndPortLateFailureLeaveBothHistoriesAndOutputsUntouched) {
  const auto p=Material();History history;Input in=Step(0,.002,p.elastic.g*5./6.);Result value;
  native::SectionInput input;input.young=p.young_pa;input.nu=p.poisson_ratio;input.rho=p.density_kg_m3;
  input.gs=in.transverse_shear_modulus;input.reference_thickness=input.reported_thickness=.002;
  native::SectionResult result;result.thickness=.007;value.reported_thickness=.007;
  const auto before=Bytes(result);const auto port_before=Bytes(value);
  input.stress[12]=std::numeric_limits<double>::max();history.point[2].stress[2]=input.stress[12];
  for(double& x:in.strain_curvature_increment)x=0;input.increment[2]=in.strain_curvature_increment[2]=1.e297;
  const auto native_history=Bytes(input);const auto port_history=Bytes(history);
  EXPECT_FALSE(native::Evaluate(input,&result));EXPECT_FALSE(sec::UpdateShellLayeredLaw1(p,history,in,value));
  EXPECT_EQ(Bytes(result),before);EXPECT_EQ(Bytes(value),port_before);
  EXPECT_EQ(Bytes(input),native_history);EXPECT_EQ(Bytes(history),port_history);
  input.stress={};history={};input.increment[2]=in.strain_curvature_increment[2]=.001;
  ASSERT_TRUE(native::Evaluate(input,&result));ASSERT_TRUE(sec::UpdateShellLayeredLaw1(p,history,in,value));
  for(unsigned c=0;c<5;++c)NativeClose(value.material_stress[c],result.force[c]);
}
} // namespace layered_law1_test
