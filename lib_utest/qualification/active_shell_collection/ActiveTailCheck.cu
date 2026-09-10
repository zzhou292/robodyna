#include "ActiveCollectionFixture.h"
#include "lib_src/elements/qeph/QephLayeredJ2.h"
#include "lib_src/elements/t3/T3LayeredJ2.h"

namespace active_shell_test {
namespace {
tl::math::Vec3 Vector(const std::vector<double>& x,std::size_t n) { return {x[3*n],x[3*n+1],x[3*n+2]}; }
void Near(double a,double b) { EXPECT_NEAR(a,b,2e-10*(1+std::abs(b))); }
template<class Trial> void Forces(const Trial& a,const Trial& b) {
  for(std::size_t n=0;n<std::size(a.internal_force);++n) {
    Near(a.internal_force[n].x,b.internal_force[n].x); Near(a.internal_force[n].y,b.internal_force[n].y); Near(a.internal_force[n].z,b.internal_force[n].z);
    Near(a.internal_couple[n].x,b.internal_couple[n].x); Near(a.internal_couple[n].y,b.internal_couple[n].y); Near(a.internal_couple[n].z,b.internal_couple[n].z);
  }
}
}
void CheckTailHost(const Rig& r,const Prepared& p,const Results& base,const Results& next) {
  // Capacity/parameter-offset oracle: reuse the already qualified value
  // operations directly on host inputs, outside resident storage and kernels.
  // This supplements the existing independent native formulation suites.
  q::PrescribedInterval qi; qi.base_time=p.view.base_time; qi.dt=H; qi.sample_index=r.owner.accepted().epoch+1;
  t::PrescribedInterval ti; ti.base_time=qi.base_time; ti.dt=H; ti.sample_index=qi.sample_index;
  for(unsigned n=0;n<4;++n) {
    const auto global=r.binding.qeph_nodes(QCount-1)[n];
    qi.position_endpoint[n]=Vector(p.endpoint.x,global); qi.velocity_midpoint[n]=Vector(p.endpoint.v,global); qi.omega_midpoint[n]=Vector(p.endpoint.w,global);
  }
  for(unsigned n=0;n<3;++n) {
    const auto global=r.binding.t3_nodes(TCount-1)[n];
    ti.position[n]=Vector(p.endpoint.x,global); ti.velocity[n]=Vector(p.endpoint.v,global); ti.angular_velocity[n]=Vector(p.endpoint.w,global);
  }
  if(r.plastic) {
    fe::sections::PointParameters qp,tp;
    ASSERT_TRUE(r.catalog.Parameters(fe::ShellBindingFamily::Qeph,QCount-1,&qp));
    ASSERT_TRUE(r.catalog.Parameters(fe::ShellBindingFamily::T3,TCount-1,&tp));
    q::LayeredJ2ForceTrial qr; t::LayeredJ2ForceTrial tr;
    ASSERT_EQ(q::EvaluateLayeredJ2Force(r.binding.qeph_reference(QCount-1),qp,
      {base.qr.back().proposed_history,base.qs.back().history},qi,qr),q::Status::kSuccess);
    ASSERT_EQ(t::EvaluateLayeredJ2Force(r.binding.t3_reference(TCount-1),tp,
      {base.tr.back().proposed_history,base.ts.back().history},ti,tr),t::Status::kSuccess);
    Forces(next.qr.back(),qr.force); Forces(next.tr.back(),tr.force);
    Near(next.qs.back().diagnostics.maximum_plastic_strain,qr.section_diagnostics.maximum_plastic_strain);
    Near(next.ts.back().diagnostics.maximum_plastic_strain,tr.section_diagnostics.maximum_plastic_strain);
  } else {
    q::ForceTrial qr; t::ForceTrial tr;
    ASSERT_EQ(q::EvaluateForce(r.binding.qeph_reference(QCount-1),base.qr.back().proposed_history,qi,qr),q::Status::kSuccess);
    ASSERT_EQ(t::EvaluateForce(r.binding.t3_reference(TCount-1),base.tr.back().proposed_history,ti,tr),t::Status::kSuccess);
    Forces(next.qr.back(),qr); Forces(next.tr.back(),tr);
  }
}
}
