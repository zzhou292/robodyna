// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../solid6z_force/TestSupport.h"
#include "../solid6z_force/Compare.h"
#include "../solid_law42_caller/Values.h"
#include "../controlled_hourglass/WorkComparison.h"
#include "lib_src/elements/solid6z/Solid6zForceMaterial.h"
#include "lib_src/elements/solid6z/controlled_hourglass/Response.h"
#include <vector>
extern "C" void s6_control_native_caller(const double*,const double*,const double*,const double*,const double*,const double*,const double*,double*,double*,double*,double*,double*,int*);
extern "C" void s6_control_initial_native(const double*,const double*,const double*,const double*,const double*,double*,double*,double*,double*,double*,int*);
namespace s6_control_test {
namespace s=tl::fea::solid6z;namespace c=s::controlled_hourglass;namespace b=tl::fea::solid_common;
namespace legacy=solid6z_force_test;
struct Case {s::Reference reference;s::Material material;s::PrescribedInterval interval;c::HistoryValues accepted;bool initialization=false;};
struct Trial {s::CurrentGeometry geometry;tl::material::law42::CallerResult material;c::Result result;};
TL_BRICK_HD inline s::Status Evaluate(const Case& x,Trial& output) {
  if(!x.reference.prepared())return s::Status::InvalidInput;
  Trial next;auto status=s::force_detail::Current(x.reference,x.interval,next.geometry);if(status!=s::Status::Success)return status;
  status=s::force_detail::EvaluateMaterial(x.reference,x.material,next.geometry,x.interval.dt_s,x.accepted.material,x.initialization,next.material);
  if(status!=s::Status::Success)return status;
  status=c::Evaluate(x.reference,x.material,next.geometry,next.material,x.interval.dt_s,x.accepted.controlled_hourglass,next.result);
  if(status!=s::Status::Success)return status;output=next;return s::Status::Success;
}
inline Case Base(s::ReferenceInput input=legacy::ReferenceInput()) {
  Case x;x.reference=legacy::Reference(input);x.material=legacy::Material(input.density_kg_m3);x.accepted.material.density_kg_m3=x.material.density_kg_m3;
  x.interval.dt_s=1e-6;for(unsigned n=0;n<6;++n)x.interval.position_endpoint_m[n]=input.position_m[n];return x;
}
inline std::array<double,98> Geometry(const s::CurrentGeometry& g) {
  std::array<double,98>a{};unsigned i=0;for(double v:g.frame.v)a[i++]=v;
  for(const auto& v:g.local_position_m){a[i++]=v.x;a[i++]=v.y;a[i++]=v.z;}
  for(const auto& v:g.local_velocity_m_s){a[i++]=v.x;a[i++]=v.y;a[i++]=v.z;}
  for(const auto& row:g.point_gradient_per_m)for(double v:row)a[i++]=v;
  for(double v:g.world_displacement_gradient)a[i++]=v;for(double v:g.material_displacement_gradient)a[i++]=v;
  for(double v:g.velocity_gradient_per_s)a[i++]=v;for(double v:g.engineering_rate_per_s)a[i++]=v;
  a[i++]=g.current_volume_m3;a[i++]=g.characteristic_length_m;return a;
}
inline std::array<double,21> History(const c::HistoryValues& h) {
  std::array<double,21>a{};unsigned i=0;for(double v:h.material.stress_pa)a[i++]=v;
  a[i++]=h.material.density_kg_m3;a[i++]=h.material.internal_energy_density_j_m3;a[i++]=h.material.bulk_pressure_pa;
  for(const auto& row:h.controlled_hourglass.force_n)for(double v:row)a[i++]=v;return a;
}
inline std::array<double,63> Hourglass(const c::hg::Result& h) {
  std::array<double,63>a{};unsigned i=0;for(const auto& row:h.proposed_state.force_n)for(double v:row)a[i++]=v;
  for(const auto& v:h.local_force_n){a[i++]=v.x;a[i++]=v.y;a[i++]=v.z;}
  a[i++]=h.internal_energy_density_j_m3;a[i++]=h.raw_stiffness_n_m;a[i++]=h.work_j;
  for(const auto& row:h.modal_velocity_m_s)for(double v:row)a[i++]=v;
  for(const auto& row:h.modal_force_n)for(double v:row)a[i++]=v;return a;
}
inline std::vector<double> Values(const Trial& t) {
  const auto g=Geometry(t.geometry);std::vector<double>a(g.begin(),g.end());const auto m=law42_caller_test::Values(t.material);a.insert(a.end(),m.begin(),m.end());
  const auto h=History(t.result.proposed_values);a.insert(a.end(),h.begin(),h.end());const auto hg=Hourglass(t.result.expanded_hourglass);a.insert(a.end(),hg.begin(),hg.end());
  for(const auto& row:t.result.native_projection)for(double v:row)a.push_back(v);
  for(const auto* array:{t.result.material_local_force_n,t.result.local_force_n,t.result.world_native_force_n,t.result.world_force_n})for(unsigned n=0;n<6;++n){a.push_back(array[n].x);a.push_back(array[n].y);a.push_back(array[n].z);}
  a.push_back(t.result.raw_stin_n_m);return a;
}
struct NativeResult {std::array<double,98>geometry{};std::array<double,33>material{};std::array<double,21>history{};std::array<double,54>forces{};std::array<double,44>control{};int status=-1;};
struct NativeHistory {
  std::array<double,4>parameters{};std::array<double,18>original{};std::array<double,11>reference{};std::array<double,21>accepted{};std::array<int,6>permutation{};
  explicit NativeHistory(const Case& x){const auto r=solid6z_test::Native(x.reference.input());if(r.status)throw std::runtime_error("native reference failed");
    permutation=r.permutation;parameters={x.material.mu_pa,x.material.poisson_ratio,x.material.density_kg_m3,x.material.tension_cutoff_pa};
    for(unsigned n=0;n<6;++n){const auto p=x.reference.input().position_m[permutation[n]];original[3*n]=p.x;original[3*n+1]=p.y;original[3*n+2]=p.z;}
    std::copy_n(r.values.begin()+27,10,reference.begin());reference[10]=r.values[37];accepted=History(x.accepted);
  }
  NativeResult Step(const Case& x)const {
    NativeResult r;double positions[18],velocities[18];
    for(unsigned n=0;n<6;++n){const auto p=x.interval.position_endpoint_m[permutation[n]],v=x.interval.velocity_midpoint_m_s[permutation[n]];
      positions[3*n]=p.x;positions[3*n+1]=p.y;positions[3*n+2]=p.z;velocities[3*n]=v.x;velocities[3*n+1]=v.y;velocities[3*n+2]=v.z;}
    const double step[]{x.interval.dt_s,.1,1};
    if(x.initialization)s6_control_initial_native(parameters.data(),original.data(),reference.data(),velocities,step+1,r.geometry.data(),r.material.data(),r.history.data(),r.forces.data(),r.control.data(),&r.status);
    else s6_control_native_caller(parameters.data(),original.data(),reference.data(),accepted.data(),positions,velocities,step,r.geometry.data(),r.material.data(),r.history.data(),r.forces.data(),r.control.data(),&r.status);
    const auto raw=r.forces;const auto control=r.control;
    for(unsigned n=0;n<6;++n){for(unsigned k=0;k<3;++k)r.forces[36+3*permutation[n]+k]=raw[36+3*n+k];r.control[38+permutation[n]]=control[38+n];}return r;
  }
};
inline void Compare(const Case& x,const Trial& a,const NativeResult& n) {
  ASSERT_EQ(n.status,0);const auto g=Geometry(a.geometry);const unsigned ge[]{0,9,27,45,63,72,81,90,96,97,98};
  for(unsigned i=0;i<10;++i)EXPECT_TRUE(legacy::Group("geometry",g.data(),n.geometry.data(),ge[i],ge[i+1]));
  const auto m=law42_caller_test::Values(a.material);double strain=1;
  for(unsigned i=22;i<28;++i)strain+=std::max(std::abs(m[i]),std::abs(n.material[i]));
  const double stress_roundoff=128*std::numeric_limits<double>::epsilon()*(x.material.bulk_pa+2*x.material.mu_pa)*strain;
  const unsigned me[]{0,6,7,8,9,15,17,18,19,20,21,22,28,29,30,31,32,33};
  for(unsigned i=0;i<17;++i)EXPECT_TRUE(legacy::Group("material",m.data(),n.material.data(),me[i],me[i+1],(me[i]==0||me[i]==9||me[i]==15)?stress_roundoff:0));
  const auto h=History(a.result.proposed_values);const unsigned he[]{0,6,7,8,9,21};
  for(unsigned i=0;i<5;++i)EXPECT_TRUE(legacy::Group("history",h.data(),n.history.data(),he[i],he[i+1],he[i]==0?stress_roundoff:0));
  double gradient=0;for(unsigned i=45;i<63;++i)gradient+=std::abs(n.geometry[i]);const double force_roundoff=stress_roundoff*n.geometry[96]*gradient;
  double forces[54];unsigned i=0;for(const auto* array:{a.result.material_local_force_n,a.result.local_force_n,a.result.world_force_n})for(unsigned node=0;node<6;++node)for(unsigned k=0;k<3;++k)forces[i++]=b::Component(array[node],k);
  for(unsigned group=0;group<3;++group)EXPECT_TRUE(legacy::Group("force",forces,n.forces.data(),18*group,18*(group+1),force_roundoff));
  const auto hg=Hourglass(a.result.expanded_hourglass);std::array<double,63> nh{};nh[38]=n.control[24];std::copy_n(n.control.begin(),24,nh.begin()+39);
  EXPECT_TRUE(legacy::Group("modal",hg.data(),nh.data(),39,51));EXPECT_TRUE(legacy::Group("amplitude",hg.data(),nh.data(),51,63));
  EXPECT_TRUE(controlled_test::SignedWorkMatches(hg,nh,x.interval.dt_s));
  double p[12];i=0;for(const auto& row:a.result.native_projection)for(double v:row)p[i++]=v;
  EXPECT_TRUE(legacy::Group("projection",p,n.control.data()+26,0,12));
  EXPECT_NEAR(a.result.raw_stin_n_m,n.control[25],3e-10*std::abs(n.control[25]));
  for(unsigned node=0;node<6;++node)EXPECT_DOUBLE_EQ(n.control[38+node],(1./3.)*n.control[25]);
  EXPECT_FALSE(c::NativeGeometricDistortionEnabled);
}
inline Trial Check(const Case& x){Trial r;EXPECT_EQ(Evaluate(x,r),s::Status::Success);Compare(x,r,NativeHistory(x).Step(x));return r;}
inline std::vector<Case> Cases(){std::vector<Case> out{Base()};auto x=Base();x.interval=legacy::Path(x.reference,10);out.push_back(x);
  for(unsigned k=0;k<3;++k)for(unsigned h=0;h<4;++h){x=Base();x.accepted.controlled_hourglass.force_n[k][h]=.125*(1+4*k+h);out.push_back(x);}
  auto source=legacy::ReferenceInput();std::swap(source.position_m[0],source.position_m[3]);std::swap(source.position_m[1],source.position_m[4]);std::swap(source.position_m[2],source.position_m[5]);
  std::swap(source.source_node_id[0],source.source_node_id[3]);std::swap(source.source_node_id[1],source.source_node_id[4]);std::swap(source.source_node_id[2],source.source_node_id[5]);
  x=Base(source);x.interval=legacy::Path(x.reference,5);out.push_back(x);return out;
}
} // namespace s6_control_test
