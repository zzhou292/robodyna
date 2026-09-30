#include "Law1TestSupport.h"
namespace layered_law1_test {
TEST(LayeredLaw1Values, IndependentLongDoublePointWithActualPhysicalThickness) {
  for(double e:{200e9,10e9}) {
    const auto p=Material(e);PointHistory h{{1e6,-2e6,3e6,-4e6,5e6}};
    PointInput in{{.003,-.001,.0007,-.0005,.0002},p.elastic.g*5./6.,.0005,.002};PointResult out;
    ASSERT_TRUE(mat::UpdateShellElasticLaw1Point(p,h,in,out));
    const long double E=e,nu=.3L,G=E/(2*(1+nu)),A=E/(1-nu*nu),B=nu*A;
    Near(out.history.stress[0],h.stress[0]+A*in.strain_increment[0]+B*in.strain_increment[1]);
    Near(out.history.stress[1],h.stress[1]+B*in.strain_increment[0]+A*in.strain_increment[1]);
    Near(out.history.stress[2],h.stress[2]+G*in.strain_increment[2]);
    for(unsigned c=3;c<5;++c)Near(out.history.stress[c],h.stress[c]+(long double)in.transverse_shear_modulus*in.strain_increment[c]);
    const long double ezz=-nu*(in.strain_increment[0]+(long double)in.strain_increment[1])/(1-nu);
    Near(out.reported_thickness,in.reported_thickness+ezz*in.layer_thickness,1.e-3L);
    EXPECT_NE(out.reported_thickness,in.reported_thickness);
  }
}
TEST(LayeredLaw1Values, IndependentSectionBendingWeightsAndNoPlasticState) {
  const auto p=Material();History h;Input in;in.reference_thickness=in.reported_thickness=.002;
  in.transverse_shear_modulus=p.elastic.g*5./6.;in.strain_curvature_increment[5]=.2;
  Result out;ASSERT_TRUE(sec::UpdateShellLayeredLaw1(p,h,in,out));
  const long double E=p.young_pa,nu=p.poisson_ratio,A=E/(1-nu*nu);
  Near(out.material_stress[0],0);Near(out.material_stress[1],0);
  const long double wm=static_cast<double>(0.0833333f);
  const long double exact=A*.2L*.002L*wm;
  Near(out.bending_stress[0],exact);Near(out.bending_stress[1],nu*exact);
  EXPECT_GT(std::abs(out.bending_stress[0]-double(A*.2L*.002L/12)),.1);
  EXPECT_EQ(sizeof(PointHistory),5*sizeof(double));
}
TEST(LayeredLaw1Values, ThreePointLateOverflowAndInvalidCoefficientsPreserveOutputsAndRetry) {
  const auto p=Material();History h;Input in=Step(0,.002,p.elastic.g*5./6.);Result out;
  ASSERT_TRUE(sec::UpdateShellLayeredLaw1(p,h,in,out));const auto before=Bytes(out);
  auto bad=h;bad.point[2].stress[2]=std::numeric_limits<double>::max();
  auto increment=in;for(double& x:increment.strain_curvature_increment)x=0;increment.strain_curvature_increment[2]=1.e297;
  EXPECT_FALSE(sec::UpdateShellLayeredLaw1(p,bad,increment,out));EXPECT_EQ(Bytes(out),before);
  auto material=p;material.elastic.a11=std::nextafter(material.elastic.a11,INFINITY);
  EXPECT_FALSE(sec::UpdateShellLayeredLaw1(material,h,in,out));EXPECT_EQ(Bytes(out),before);
  PointResult point;point.reported_thickness=.007;const auto point_before=Bytes(point);
  PointInput pi;pi.transverse_shear_modulus=p.elastic.g;pi.layer_thickness=.001;pi.reported_thickness=.002;
  pi.strain_increment[0]=10;
  EXPECT_FALSE(mat::UpdateShellElasticLaw1Point(p,{},pi,point));EXPECT_EQ(Bytes(point),point_before);
  Parameters params=p;const auto prepared=Bytes(params);
  EXPECT_FALSE(mat::PrepareShellElasticLaw1Point(1.,.5,1.,params));EXPECT_EQ(Bytes(params),prepared);
  ASSERT_TRUE(sec::UpdateShellLayeredLaw1(p,h,in,out));EXPECT_EQ(Bytes(out),before);
}
} // namespace layered_law1_test
