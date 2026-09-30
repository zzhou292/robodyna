#include "Tab1NativeSupport.h"
#include <vector>
namespace tab1_test {
extern "C" void legacy_layered_failure_caller(int,int,const double*,const double*,const double*,
    const double*,const double*,const double*,const double*,const double*,const double*,
    const double*,const double*,double*,double*,double*,double*,double*,double*,double*,
    double*,double*,double*,int*);
namespace {
void LegacyStep(const sec::PointParameters& p,double failure_strain,
    const sec::ShellLayeredJ2Input& in,double time,double area,double dm,
    layered_failure_test::NativeState& n,NativeTrace& trace) {
  const bool table=p.hardening==mat::ShellPlasticityHardeningKind::Tabulated;
  const int count=table?static_cast<int>(p.curve.count):0;
  std::vector<double> curve(2*(count+1),0);
  for(int i=0;i<count;++i) {
    curve[2*(i+1)]=p.curve.plastic_strain[i];
    curve[2*(i+1)+1]=p.curve.yield_stress_pa[i];
  }
  const double basic[]{p.density_kg_m3,p.young_pa,p.poisson_ratio,in.transverse_shear_modulus};
  const double linear[]{p.linear.initial_yield_pa,p.linear.tangent_modulus_pa};
  const double rate[]{p.rate.cowper_symonds_c_per_s,p.rate.cowper_symonds_p,p.rate.cutoff_hz};
  const double reference=n.thickness;
  legacy_layered_failure_caller(table?1:0,count,curve.data(),basic,linear,rate,&failure_strain,
      &in.dt,&time,in.strain_curvature_increment,&reference,&area,&dm,n.points.data(),n.failures.data(),
      &n.parent,n.material.data(),n.stress.data(),n.moment.data(),&n.thickness,n.work.data(),
      trace.point_values.data(),trace.diagnostics.data(),&trace.removed);
}
}
TEST(Tab1Native, RefactoredDefaultJohnsonCallerMatchesFrozenAllHistoryWorkAndPhaseBits) {
  for(unsigned mode=0;mode<3;++mode) for(unsigned mask=0;mask<8;++mask) {
    SCOPED_TRACE(mode);
    SCOPED_TRACE(mask);
    const auto p=layered_failure_test::Parameters(mode==2,mode!=0);
    auto current=layered_failure_test::NativeSeed(layered_failure_test::Seed(mask));
    auto previous=current;
    unsigned removed=0;
    for(unsigned step=0;step<18;++step) {
      auto in=layered_failure_test::Input(p);
      in.strain_curvature_increment[0]=step<14?.003:-.00001;
      in.strain_curvature_increment[1]=-.15*in.strain_curvature_increment[0];
      in.strain_curvature_increment[2]=.0001;
      in.strain_curvature_increment[4]=-.00001;
      in.strain_curvature_increment[5]=.12;
      in.strain_curvature_increment[7]=-.03;
      NativeTrace a,b;
      layered_failure_test::NativeStep(p,.005,in,(step+1)*in.dt,.01,.02,current,a);
      LegacyStep(p,.005,in,(step+1)*in.dt,.01,.02,previous,b);
      EXPECT_EQ(Bytes(current.points),Bytes(previous.points));
      EXPECT_EQ(Bytes(current.failures),Bytes(previous.failures));
      EXPECT_EQ(Bytes(current.parent),Bytes(previous.parent));
      EXPECT_EQ(Bytes(current.material),Bytes(previous.material));
      EXPECT_EQ(Bytes(current.stress),Bytes(previous.stress));
      EXPECT_EQ(Bytes(current.moment),Bytes(previous.moment));
      EXPECT_EQ(Bytes(current.thickness),Bytes(previous.thickness));
      EXPECT_EQ(Bytes(current.work),Bytes(previous.work));
      EXPECT_EQ(Bytes(a.point_values),Bytes(b.point_values));
      EXPECT_EQ(Bytes(a.diagnostics),Bytes(b.diagnostics));
      EXPECT_EQ(a.removed,b.removed);
      removed+=a.removed;
    }
    EXPECT_EQ(removed,mask==7?0u:1u);
  }
}
} // namespace tab1_test
