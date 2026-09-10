#include "SourcePartT3MaterialOracle.h"

namespace crash::cases::source_part_elastic::test::t3_material {
using namespace native;
using native::force_test::Oracle;
using native::force_test::Near;
// Same independent continuum equations and unchanged Near tolerances as the
// pinned T3ForceTestOracle. Only the layer boundary is different: the native
// and CUDA trajectories each supply their actual raw rates and accepted base.
Oracle Independent(const ReferenceInput& r,const HistoryValues& base,const PrescribedInterval& in,const Kinematics& geometry) {
  Oracle out; const long double area=geometry.area;
  const long double E=r.young_modulus,nu=r.poisson_ratio,t=r.thickness;
  const long double g=E/(2*(1+nu)),a11=E/(1-nu*nu),a12=nu*a11,gs=g*5/6;
  std::array<long double,8> d{};
  for(unsigned k=0;k<8;++k) { d[k]=static_cast<long double>(geometry.raw_rate[k])*in.dt/area; out.values[13+k]=base.strain_curvature[k]+d[k]; }
  std::array<long double,5> material{base.material_stress[0]+a11*d[0]+a12*d[1],
      base.material_stress[1]+a12*d[0]+a11*d[1],base.material_stress[2]+g*d[2],
      base.material_stress[3]+gs*d[3],base.material_stress[4]+gs*d[4]};
  const std::array<long double,3> moment{base.bending_stress[0]+t/12*(a11*d[5]+a12*d[6]),
      base.bending_stress[1]+t/12*(a12*d[5]+a11*d[6]),base.bending_stress[2]+t/12*g*d[7]};
  const long double inverse_dt=in.dt/std::max(static_cast<long double>(in.dt)*in.dt,1e-20L);
  const long double visc=1.414L*.015L*r.density*std::sqrt(E/std::max(static_cast<long double>(r.density),1e-20L))
      *std::sqrt(area)*inverse_dt;
  auto total=material;
  total[0]+=visc*(d[0]+.5L*d[1]); total[1]+=visc*(d[1]+.5L*d[0]); total[2]+=visc*d[2]/3;
  long double membrane=0,bending=0;
  for(unsigned k=0;k<5;++k) {
    out.values[k]=total[k]; out.values[k+5]=material[k]; membrane+=(base.stress[k]+total[k])*d[k];
  }
  for(unsigned k=0;k<3;++k) { out.values[10+k]=moment[k]; bending+=(base.bending_stress[k]+moment[k])*d[k+5]; }
  out.values[21]=base.thickness*(1-nu*(d[0]+d[1])/(1-nu));
  out.values[22]=base.internal_work[0]+membrane*.5L*t*area;
  out.values[23]=base.internal_work[1]+bending*.5L*t*t*area;
  out.values[24]=std::sqrt((d[5]*d[5]+d[6]*d[6]+d[5]*d[6]+d[7]*d[7]/4)*base.thickness*base.thickness/9+
      4.L/3*(d[0]*d[0]+d[1]*d[1]+d[0]*d[1]+d[2]*d[2]/4))*inverse_dt;
  out.values[25]=1;
  return out;
}
void Check(const Reference& r,const HistoryValues& base,const PrescribedInterval& in,const ForceTrial& out) {
  kinematic_test::Check(out.kinematics,in);
  std::array<double,26> actual{}; detail::PackHistory(out.proposed_history.data(),actual);
  const auto expected=Independent(r.data().input,base,in,out.kinematics);
  for(unsigned k=0;k<26;++k) { SCOPED_TRACE(k); Near(actual[k],expected.values[k],k==22||k==23?2e-22L:2e-11L); }
  for(unsigned k=0;k<2;++k) Near(out.diagnostics.internal_work_increment[k],
      expected.values[22+k]-base.internal_work[k],2e-22L+64*std::numeric_limits<double>::epsilon()*
      (std::abs(base.internal_work[k])+std::abs(out.proposed_history.data().internal_work[k])));
  EXPECT_EQ(out.diagnostics.effective_thickness,r.data().input.thickness);
  EXPECT_EQ(out.proposed_history.stamp().time,in.base_time+in.dt);
  EXPECT_EQ(out.proposed_history.stamp().sample_index,in.sample_index);
}
} // namespace crash::cases::source_part_elastic::test::t3_material
