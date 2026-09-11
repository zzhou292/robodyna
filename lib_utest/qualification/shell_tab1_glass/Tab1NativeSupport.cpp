#include "Tab1NativeSupport.h"
namespace tab1_test {
extern "C" void tab1_layered_caller(const double*,const double*,const double*,const double*,
    const double*,const double*,const double*,const double*,const double*,const double*,
    double*,double*,double*,double*,double*,double*,double*,double*,double*,double*,int*);
void NativeStep(const sec::ShellLayeredJ2Input& in,double time,double area,double dm,
    NativeState& n,NativeTrace& trace) {
  const double basic[]{2500,70e9,.22,in.transverse_shear_modulus};
  const double linear[]{30e6,1e9},rate[]{0,1,10000},table[]{-.3,0,.3,.015};
  const double reference_thickness=n.thickness;
  tab1_layered_caller(basic,linear,rate,table,&in.dt,&time,in.strain_curvature_increment,
      &reference_thickness,&area,&dm,n.points.data(),n.failures.data(),&n.parent,
      n.material.data(),n.stress.data(),n.moment.data(),&n.thickness,n.work.data(),
      trace.point_values.data(),trace.diagnostics.data(),&trace.removed);
}
void Compare(const sec::ShellLayeredTab1Result& actual,const Work& work,
    const NativeState& n,const NativeTrace& trace,double thickness,double area) {
  EXPECT_EQ(actual.history.element_active,n.parent==1);
  EXPECT_EQ(actual.removed_now,trace.removed==1);
  double mean_tangent=0,minimum_tangent=1,mean_yield=0;
  for(unsigned p=0;p<3;++p) {
    SCOPED_TRACE(p);
    for(unsigned c=0;c<5;++c) {
      Close(actual.history.saved.point[p].stress[c],n.points[7*p+c],1.e-8);
      Close(actual.history.current_force_point[p].stress[c],trace.point_values[13*p+c],1.e-8);
      Close(actual.current.history.point[p].stress[c],trace.point_values[13*p+c],1.e-8);
    }
    Close(actual.history.saved.point[p].plastic_strain,n.points[7*p+5]);
    Close(actual.history.saved.point[p].filtered_rate_per_s,n.points[7*p+6],1.e-10);
    Close(actual.constitutive_increment[p],trace.point_values[13*p+6]);
    const auto& f=actual.history.failure[p];
    Close(f.damage,n.failures[5*p]);
    EXPECT_EQ(f.failure_time_s,n.failures[5*p+1]);
    EXPECT_EQ(f.point_active,n.failures[5*p+2]==1);
    Close(f.maximum_damage,n.failures[5*p+3]);
    EXPECT_EQ(f.table_segment,n.failures[5*p+4]);
    const double weight=p==1?.5:.25;
    mean_tangent+=weight*trace.point_values[13*p+7];
    minimum_tangent=std::min(minimum_tangent,trace.point_values[13*p+7]);
    mean_yield+=weight*trace.point_values[13*p+9];
  }
  for(unsigned c=0;c<5;++c) {
    Close(actual.current.material_stress[c],n.material[c],1.e-8);
    Close(work.stress[c],n.stress[c],1.e-8);
    Close(work.material_stress[c],n.material[c],1.e-8);
  }
  for(unsigned c=0;c<3;++c) {
    Close(actual.current.bending_stress[c],n.moment[c],1.e-8);
    Close(work.bending_stress[c],n.moment[c],1.e-8);
  }
  Close(actual.current.reported_thickness,n.thickness);
  Close(work.thickness,n.thickness);
  for(unsigned c=0;c<2;++c) Close(work.internal_work[c],n.work[c],1.e-12);
  const auto& d=actual.current.diagnostics;
  Close(d.plastic_work_density_increment*thickness*area,trace.diagnostics[0],1.e-12);
  Close(d.mean_plastic_strain,trace.diagnostics[1]);
  Close(d.maximum_plastic_strain,trace.diagnostics[2]);
  Close(d.mean_tangent_ratio,trace.diagnostics[3]);
  Close(d.minimum_tangent_ratio,trace.diagnostics[4]);
  Close(d.mean_yield_before_pa,trace.diagnostics[5],1.e-8);
  Close(d.last_point_yield_before_pa,trace.diagnostics[6],1.e-8);
  Close(d.mean_tangent_ratio,mean_tangent);
  Close(d.minimum_tangent_ratio,minimum_tangent);
  Close(d.mean_yield_before_pa,mean_yield,1.e-8);
}
} // namespace tab1_test
