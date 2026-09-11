// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
extern "C" void law42_native_spectrum(const double*,double*);
namespace law42_test {
TEST(Law42Native,CompleteRoutineFiniteStrainAndRepeatedRoots) {
  const auto p=Material();
  for(double a:{.6,.9,1.0,1.001,1.3,2.0}) {
    SCOPED_TRACE(a);
    NativeCompare(p,Stretch(a,a,a));
    NativeCompare(p,Stretch(a,1,1));
    NativeCompare(p,Rotate(Stretch(a,.83,1.1),.37));
  }
}
TEST(Law42Native,RemovalDensityAndPressureEnhancement) {
  NativeCompare(Material(1e6),Stretch(1.2,1.2,1.2));
  auto x=Stretch(.8,.9,1.2);x.active=0;NativeCompare(Material(),x);
  x.active=1;x.density_kg_m3=4000;NativeCompare(Material(),x);
  NativeCompare(Material(1e26,.4999),Stretch(.001,.8,.9));
}
TEST(Law42Native,OriginalWorkingUnitMaterialMapping) {
  const auto p=Material();const auto x=Rotate(Stretch(1.3,.8,1.1),.71);
  law::Parameters working;
  ASSERT_EQ(law::Prepare(24,.463,1.98e-9,1e20,working),law::Status::Ok);
  auto wx=x;wx.density_kg_m3*=1e-12;
  double native[13];Native(working,wx,native);
  for(unsigned k=0;k<8;++k)native[k]*=1e6;
  native[9]*=1e-3;native[12]*=1e6;
  law::Result r;ASSERT_EQ(law::Update(p,x,r),law::Status::Ok);
  double actual[13];Pack(p,r,actual);Compare(actual,native);
}
TEST(Law42Native,RepeatedRootResidualAndComparisonRejectsPhysicalPerturbation) {
  const double strain[6]{.9*.9-1,0,0,0,0,0};
  double native[12];law42_native_spectrum(strain,native);
  const double bound=2*std::sqrt(std::numeric_limits<double>::epsilon())*.19;
  double maximum_residual=0;
  for(unsigned j=0;j<3;++j)for(unsigned i=0;i<3;++i) {
    const double vector=native[3+3*j+i];
    const double residual=(strain[i]-native[j])*vector;
    maximum_residual=std::max(maximum_residual,std::abs(residual));
  }
  EXPECT_LT(maximum_residual,bound);
  EXPECT_GT(maximum_residual,1e-12); // Native double-root split is observable.
  tl::math::SymmetricSpectrum3 spectrum;
  ASSERT_TRUE(tl::math::SymmetricEigen3(strain,spectrum));
  for(unsigned j=0;j<3;++j)for(unsigned i=0;i<3;++i)
    EXPECT_LT(std::abs((strain[i]-spectrum.value[j])*spectrum.vectors.v[3*i+j]),bound);
  const auto p=Material();law::Parameters changed;
  ASSERT_EQ(law::Prepare(p.mu_pa*1.00001,p.poisson_ratio,p.density_kg_m3,p.tension_cutoff_pa,changed),law::Status::Ok);
  const auto x=Stretch(1.4,.9,1.1);law::Result altered;
  ASSERT_EQ(law::Update(changed,x,altered),law::Status::Ok);
  double reference[13];Native(p,x,reference);
  const double material_scale=std::max({std::abs(reference[0]),std::abs(reference[1]),std::abs(reference[2])});
  EXPECT_GT(std::abs(altered.stress_pa[0]-reference[0]),
    (2e-10+2*std::sqrt(std::numeric_limits<double>::epsilon()))*material_scale);
}
}
