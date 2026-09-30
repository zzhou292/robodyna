#include "FailureNativeFixture.h"
#include <vector>

namespace layered_failure_test {
extern "C" void layered_failure_caller(int,int,const double*,const double*,const double*,
    const double*,const double*,const double*,const double*,const double*,const double*,
    const double*,const double*,double*,double*,double*,double*,double*,double*,double*,
    double*,double*,double*,int*);

NativeState NativeSeed(const sec::ShellLayeredJ2FailureHistory& base) {
  NativeState n;
  n.parent=base.element_active?1:0;
  for(unsigned p=0;p<3;++p) {
    std::copy_n(base.saved.point[p].stress,5,n.points.begin()+7*p);
    n.points[7*p+5]=base.saved.point[p].plastic_strain;
    n.points[7*p+6]=base.saved.point[p].filtered_rate_per_s;
    n.failures[3*p]=base.failure[p].damage;
    n.failures[3*p+1]=base.failure[p].failure_time_s;
    n.failures[3*p+2]=base.failure[p].point_active?1:0;
  }
  return n;
}

void NativeStep(const sec::PointParameters& p,double failure_strain,
    const sec::ShellLayeredJ2Input& input,double time,double area,double dm,
    NativeState& native,NativeTrace& trace) {
  const bool table=p.hardening==mat::ShellPlasticityHardeningKind::Tabulated;
  const int count=table?static_cast<int>(p.curve.count):0;
  std::vector<double> curve(2*(count+1),0);
  for(int i=0;i<count;++i) {
    curve[2*(i+1)]=p.curve.plastic_strain[i];
    curve[2*(i+1)+1]=p.curve.yield_stress_pa[i];
  }
  const double basic[]{p.density_kg_m3,p.young_pa,p.poisson_ratio,input.transverse_shear_modulus};
  const double linear[]{p.linear.initial_yield_pa,p.linear.tangent_modulus_pa};
  const double rate[]{p.rate.cowper_symonds_c_per_s,p.rate.cowper_symonds_p,p.rate.cutoff_hz};
  const double native_reference_thickness=native.thickness;
  layered_failure_caller(table?1:0,count,curve.data(),basic,linear,rate,&failure_strain,
      &input.dt,&time,input.strain_curvature_increment,&native_reference_thickness,&area,&dm,
      native.points.data(),native.failures.data(),&native.parent,native.material.data(),
      native.stress.data(),native.moment.data(),&native.thickness,native.work.data(),
      trace.point_values.data(),trace.diagnostics.data(),&trace.removed);
}
} // namespace layered_failure_test
