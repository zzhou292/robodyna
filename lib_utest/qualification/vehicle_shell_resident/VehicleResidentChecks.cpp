#include "VehicleResidentFixture.h"
#include "lib_src/elements/qeph/QephLayeredJ2.h"
#include "lib_src/elements/t3/T3LayeredJ2.h"
#include <cmath>
#include <cstdio>
#include <limits>
namespace vehicle_resident_test {
namespace {
tl::math::Vec3 Vector(const std::vector<double>& x,std::size_t n){return {x[3*n],x[3*n+1],x[3*n+2]};}
void Near(double a,double b){EXPECT_NEAR(a,b,2e-10*(1+std::abs(b)));}
template<class Trial> void Forces(const Trial& a,const Trial& b) {
  for(std::size_t n=0;n<std::size(a.internal_force);++n) {
    for(const auto pair:{std::pair{a.internal_force[n],b.internal_force[n]},std::pair{a.internal_couple[n],b.internal_couple[n]}})
      {Near(pair.first.x,pair.second.x);Near(pair.first.y,pair.second.y);Near(pair.first.z,pair.second.z);}
  }
}
template<class T> void Same(const std::vector<T>& a,const std::vector<T>& b) {
  ASSERT_EQ(a.size(),b.size());if(!a.empty())EXPECT_EQ(std::memcmp(a.data(),b.data(),a.size()*sizeof(T)),0);
}
}
void SameResults(const Results& a,const Results& b){Same(a.qr,b.qr);Same(a.tr,b.tr);Same(a.qs,b.qs);Same(a.ts,b.ts);}
void SameSnapshot(const Snapshot& a,const Snapshot& b) {
  Same(a.x,b.x);Same(a.v,b.v);Same(a.w,b.w);Same(a.orientation,b.orientation);Same(a.reaction,b.reaction);Same(a.couple,b.couple);
  EXPECT_TRUE(fe::trial_identity::SameStamp(a.stamp,b.stamp));
}
void CheckNativeTail(const Rig& r,const Prepared& p,const Results& base,const Results& next) {
  // Capacity/offset oracle uses qualified host value mechanics, independently
  // of resident pointer rebasing and GPU indexing. Native Fortran qualification
  // remains the existing independent formulation suite.
  q::PrescribedInterval qi;qi.base_time=p.view.base_time;qi.dt=H;qi.sample_index=r.owner.accepted().epoch+1;
  t::PrescribedInterval ti;ti.base_time=qi.base_time;ti.dt=H;ti.sample_index=qi.sample_index;
  for(unsigned n=0;n<4;++n){const auto i=r.binding.qeph_nodes(r.nq-1)[n];
    qi.position_endpoint[n]=Vector(p.endpoint.x,i);qi.velocity_midpoint[n]=Vector(p.endpoint.v,i);qi.omega_midpoint[n]=Vector(p.endpoint.w,i);}
  for(unsigned n=0;n<3;++n){const auto i=r.binding.t3_nodes(r.nt-1)[n];
    ti.position[n]=Vector(p.endpoint.x,i);ti.velocity[n]=Vector(p.endpoint.v,i);ti.angular_velocity[n]=Vector(p.endpoint.w,i);}
  if(r.plastic){
    fe::sections::PointParameters qp,tp;
    ASSERT_TRUE(r.catalog.Parameters(fe::ShellBindingFamily::Qeph,r.nq-1,&qp));
    ASSERT_TRUE(r.catalog.Parameters(fe::ShellBindingFamily::T3,r.nt-1,&tp));
    q::LayeredJ2ForceTrial qr;t::LayeredJ2ForceTrial tr;
    ASSERT_EQ(q::EvaluateLayeredJ2Force(r.binding.qeph_reference(r.nq-1),qp,{base.qr.back().proposed_history,base.qs.back().history},qi,qr),q::Status::kSuccess);
    ASSERT_EQ(t::EvaluateLayeredJ2Force(r.binding.t3_reference(r.nt-1),tp,{base.tr.back().proposed_history,base.ts.back().history},ti,tr),t::Status::kSuccess);
    Forces(next.qr.back(),qr.force);Forces(next.tr.back(),tr.force);
    Near(next.qs.back().diagnostics.maximum_plastic_strain,qr.section_diagnostics.maximum_plastic_strain);
    Near(next.ts.back().diagnostics.maximum_plastic_strain,tr.section_diagnostics.maximum_plastic_strain);
  } else {
    q::ForceTrial qr;t::ForceTrial tr;
    ASSERT_EQ(q::EvaluateForce(r.binding.qeph_reference(r.nq-1),base.qr.back().proposed_history,qi,qr),q::Status::kSuccess);
    ASSERT_EQ(t::EvaluateForce(r.binding.t3_reference(r.nt-1),base.tr.back().proposed_history,ti,tr),t::Status::kSuccess);
    Forces(next.qr.back(),qr);Forces(next.tr.back(),tr);
  }
}
void CheckCompleteFields(const Rig& r,const Prepared& p,const Results& next) {
  for(std::size_t e=0;e<r.nq;++e) {
    const auto& h=next.qr[e].proposed_history;
    ASSERT_TRUE(h.matches_reference(r.binding.qeph_reference(e)))<<e;
    ASSERT_EQ(h.stamp().sample_index,r.owner.accepted().epoch+1)<<e;ASSERT_EQ(h.stamp().time,p.view.proposed_time)<<e;
  }
  for(std::size_t e=0;e<r.nt;++e) {
    const auto& h=next.tr[e].proposed_history;
    ASSERT_TRUE(h.matches_reference(r.binding.t3_reference(e)))<<e;
    ASSERT_EQ(h.stamp().sample_index,r.owner.accepted().epoch+1)<<e;ASSERT_EQ(h.stamp().time,p.view.proposed_time)<<e;
  }
  // Independent complete-node metric reduction; no family or partition is
  // counted twice, and this diagnostic is not a global energy admission proof.
  long double kt=0,kr=0,kp=0,ka=0;
  for(std::size_t n=0;n<r.n;++n){long double vv=0,ww=0;
    for(unsigned a=0;a<3;++a){const long double v=p.endpoint.v[3*n+a],w=p.endpoint.w[3*n+a];vv+=v*v;ww+=w*w;}
    const auto& m=r.binding.nodes()[n].native;kt+=.5L*m.mass*vv;kr+=.5L*m.isotropic_inertia*ww;
    kp+=.5L*m.physical_inertia*ww;ka+=.5L*m.added_inertia*ww;}
  const double budget=(32+8*r.n)*std::numeric_limits<double>::epsilon();
  for(const auto pair:{std::pair{kt,next.diagnostics.kinetic.translation},std::pair{kr,next.diagnostics.kinetic.rotation},
      std::pair{kp,next.diagnostics.kinetic.physical_isotropic},std::pair{ka,next.diagnostics.kinetic.added_isotropic}})
    EXPECT_NEAR(pair.second,double(pair.first),budget*double(std::abs(pair.first)+std::abs(pair.second)));
  EXPECT_FALSE(next.diagnostics.qeph.kinetic_available);EXPECT_FALSE(next.diagnostics.t3.kinetic_available);
}
void ReportStorage(const Rig& r) {
  const auto owner=r.owner.allocations(),q=r.qeph.allocations(),t=r.t3.allocations(),p=r.publication.allocations();
  std::size_t binding=0,catalog=0;
  ASSERT_TRUE(fe::shell_batch_detail::RetainedScopeBytes(&r.binding,&r.catalog,true,binding,catalog));
  const auto result=r.nq*sizeof(q::ForceTrial)+r.nt*sizeof(t::ForceTrial)+(r.plastic?(r.nq+r.nt)*sizeof(fe::ShellBatchSectionState):0);
  std::printf("{\"kind\":\"vehicle_resident_actual_payload\",\"q\":%zu,\"t\":%zu,\"nodes\":%zu,\"plastic\":%s,\"owner_device_bytes\":%zu,\"qeph_device_bytes\":%zu,\"t3_device_bytes\":%zu,\"publication_device_bytes\":%zu,\"total_module_device_bytes\":%zu,\"binding_and_catalog_host_bytes_unique\":%zu,\"one_complete_results_buffer_bytes\":%zu,\"one_complete_node_snapshot_bytes\":%zu}\n",
    r.nq,r.nt,r.n,r.plastic?"true":"false",owner.device_bytes,q.device_bytes,t.device_bytes,p.device_bytes,
    owner.device_bytes+q.device_bytes+t.device_bytes+p.device_bytes,binding+catalog+sizeof(fe::ShellBatchBinding),result,19*r.n*sizeof(double));
}
} // namespace vehicle_resident_test
