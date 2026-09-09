#include "WallCoupledFixture.h"

namespace qeph_wall_test {
namespace {
using Wide=long double;
void Check(Wide actual,Wide expected,Wide terms,Wide floor,Wide uncertainty,double& maximum) {
  const Wide budget=256*std::numeric_limits<double>::epsilon()*terms+1e-12L*floor+uncertainty;
  ASSERT_TRUE(std::isfinite(actual)); ASSERT_TRUE(std::isfinite(expected)); ASSERT_TRUE(std::isfinite(budget));
  ASSERT_GE(budget,0);
  const auto error=std::abs(actual-expected);
  EXPECT_LE(error,budget)<<std::setprecision(18)<<"actual="<<actual<<" expected="<<expected<<" budget="<<budget;
  maximum=std::max(maximum,static_cast<double>(budget?error/budget:(error?INFINITY:0)));
}
// Independent long-double center-area oracle from represented reference
// diagonals; neither native projected area nor contact's area routine is used.
std::array<Wide,N> Shares(const Rig& r) {
  std::array<Wide,N> result{};
  for(unsigned e=0;e<r.count;++e) {
    const auto& x=r.element[e].reference.input.position;
    Wide a[]{static_cast<Wide>(x[2].x)-x[0].x,static_cast<Wide>(x[2].y)-x[0].y,static_cast<Wide>(x[2].z)-x[0].z};
    Wide b[]{static_cast<Wide>(x[3].x)-x[1].x,static_cast<Wide>(x[3].y)-x[1].y,static_cast<Wide>(x[3].z)-x[1].z};
    Wide square=0;
    for(unsigned i=0;i<3;++i) { const auto c=a[(i+1)%3]*b[(i+2)%3]-a[(i+2)%3]*b[(i+1)%3]; square+=c*c; }
    const Wide share=std::sqrt(square)/8;
    for(unsigned i=0;i<4;++i) result[r.element[e].nodes[i]]+=share;
  }
  return result;
}
}
void ContactLedgers(const WallRig& w,const Snapshot& base,const Trial& t,const Staged& s,ContactEvidence& evidence) {
  const auto& r=w.shell; const auto& p=t.nodal; const auto& d=s.contact;
  const auto shares=Shares(r);
  const Wide area=r.count*static_cast<Wide>(Side)*Side;
  const Wide energy_scale=Kappa*area*Preload*Preload;
  const Wide impulse_scale=Kappa*area*Preload*(4*H0),angular_scale=Side*impulse_scale;
  Wide kick=0,drift=0,base_potential=0,next_potential=0,quadratic=0,quadratic_uncertainty=0,force_error_work=0;
  Wide impulse=0,moment_y=0,moment_z=0,terms=0,moment_terms=0;
  Wide delta_momentum=0,internal_impulse=0,momentum_terms=0,total_kinetic=0,kinetic_terms=0;
  for(unsigned j=0;j<t.contact_base_result.diagnostics.node_count;++j) {
    const auto& node=t.contact_base_result.nodes[j]; const auto n=node.node;
    const Wide a=base.x[3*n],x=p.state.x[3*n],dx=x-a;
    const Wide va=base.v[3*n],v=p.state.v[3*n];
    const Wide force=node.force_world.x;
    const Wide exact_k=Kappa*shares[n],ga=std::max(0.L,a),gx=std::max(0.L,x);
    Check(-force,exact_k*ga,std::abs(force)+exact_k*ga,0,node.force.error,evidence.work_ratio);
    EXPECT_EQ(node.force_world.y,0); EXPECT_EQ(node.force_world.z,0);
    const Wide dw=p.view.kick_dt*force*(va+v)*.5L,dd=force*dx;
    kick+=dw; drift+=dd; terms+=std::abs(dw)+std::abs(dd);
    base_potential+=.5L*exact_k*ga*ga; next_potential+=.5L*exact_k*gx*gx;
    quadratic+=.5L*exact_k*dx*dx;
    quadratic_uncertainty+=.5L*std::abs(static_cast<Wide>(node.stiffness.upper)-exact_k)*dx*dx;
    force_error_work+=node.force.error*std::abs(dx);
    impulse-=p.view.kick_dt*force;
    moment_y-=p.view.kick_dt*force*base.x[3*n+2];
    moment_z+=p.view.kick_dt*force*base.x[3*n+1];
    moment_terms+=p.view.kick_dt*std::abs(force)*(std::abs(base.x[3*n+2])+std::abs(base.x[3*n+1]));
    const Wide p0=r.mass[n]*va,p1=r.mass[n]*v;
    delta_momentum+=p1-p0;
    internal_impulse+=p.view.kick_dt*(static_cast<Wide>(p.assembled.force[3*n])-force);
    momentum_terms+=std::abs(p0)+std::abs(p1)+p.view.kick_dt*std::abs(p.assembled.force[3*n]);
  }
  for(unsigned n=0;n<r.n;++n) for(unsigned a=0;a<3;++a) {
    const unsigned i=3*n+a;
    const Wide mass[]{r.mass[n],r.inertia[n]},old[]{base.v[i],base.omega[i]},next[]{p.state.v[i],p.state.omega[i]};
    for(unsigned k=0;k<2;++k) {
      const Wide before=.5L*mass[k]*old[k]*old[k],after=.5L*mass[k]*next[k]*next[k];
      total_kinetic+=after-before; kinetic_terms+=std::abs(before)+std::abs(after);
    }
  }
  Check(d.kick_work,kick,terms,energy_scale,d.kick_work_roundoff,evidence.work_ratio);
  Check(d.drift_work,drift,terms,energy_scale,d.drift_work_roundoff,evidence.work_ratio);
  Check(total_kinetic,static_cast<Wide>(s.shell.internal_kick_work)+kick,
        kinetic_terms+std::abs(kick)+std::abs(s.shell.internal_kick_work),energy_scale,
        d.kick_work_roundoff,evidence.work_ratio);
  Check(t.contact_base.potential.value,base_potential,std::abs(base_potential),0,
        t.contact_base.potential.error,evidence.work_ratio);
  Check(d.potential.value,next_potential,std::abs(next_potential),0,d.potential.error,evidence.work_ratio);
  Check(d.potential_increment,next_potential-base_potential,std::abs(base_potential)+std::abs(next_potential),
        energy_scale,t.contact_base.potential.error+d.potential.error,evidence.work_ratio);
  Check(d.conservative_defect,next_potential-base_potential+drift,
        std::abs(base_potential)+std::abs(next_potential)+std::abs(drift),energy_scale,d.work_uncertainty,evidence.defect_ratio);
  EXPECT_GE(static_cast<Wide>(d.conservative_defect)+d.work_uncertainty,0);
  EXPECT_LE(static_cast<Wide>(d.conservative_defect)-d.work_uncertainty,d.quadratic_work_upper);
  EXPECT_GE(d.quadratic_work_upper,0);
  Check(d.quadratic_work_upper,quadratic,std::abs(quadratic),0,quadratic_uncertainty,evidence.defect_ratio);
  // The mathematical bound uses all-active stiffness even during a sign change.
  const Wide analytic_defect=next_potential-base_potential+drift;
  const Wide uncertainty=d.work_uncertainty+force_error_work+
    256*std::numeric_limits<double>::epsilon()*(std::abs(base_potential)+std::abs(next_potential)+std::abs(drift));
  EXPECT_GE(analytic_defect+uncertainty,0); EXPECT_LE(analytic_defect-uncertainty,quadratic);
  Check(d.wall_kick_impulse,impulse,std::abs(impulse),impulse_scale,d.wall_kick_impulse_error,evidence.impulse_ratio);
  Check(delta_momentum+d.wall_kick_impulse,internal_impulse,
        momentum_terms+std::abs(d.wall_kick_impulse)+std::abs(internal_impulse),impulse_scale,
        d.wall_kick_impulse_error,evidence.impulse_ratio);
  Check(d.wall_kick_moment.y,moment_y,moment_terms,angular_scale,d.wall_kick_moment_error.y,evidence.impulse_ratio);
  Check(d.wall_kick_moment.z,moment_z,moment_terms,angular_scale,d.wall_kick_moment_error.z,evidence.impulse_ratio);
  EXPECT_EQ(d.wall_kick_moment.x,0);
}
void Controls(const WallRig& w,const Snapshot& base,const Trial& t,const Staged& s,
              const Loads& contact,ContactEvidence& evidence) {
  if(!base.stamp.epoch) return;
  const auto& r=w.shell; const auto& p=t.nodal;
  const auto shell_ratio=WrongKickSeparation(r,base,p,contact,false);
  if(shell_ratio>evidence.omitted_shell) { evidence.omitted_shell=shell_ratio; evidence.omitted_shell_epoch=base.stamp.epoch; }
  Loads premature;
  for(unsigned e=0;e<r.count;++e) for(unsigned i=0;i<4;++i) {
    const auto n=r.element[e].nodes[i]; const auto f=s.elements[e].internal_force[i],c=s.elements[e].internal_couple[i];
    const double force[]{f.x,f.y,f.z},couple[]{c.x,c.y,c.z};
    for(unsigned a=0;a<3;++a) { premature.force[3*n+a]-=force[a]; premature.couple[3*n+a]-=couple[a]; }
  }
  for(unsigned j=0;j<s.contact_result.diagnostics.node_count;++j) {
    const auto& node=s.contact_result.nodes[j]; premature.force[3*node.node]+=node.force_world.x;
  }
  for(unsigned n=0;n<r.n;++n) for(unsigned a=0;a<3;++a) {
    const unsigned i=3*n+a;
    const double omitted=base.v[i]+p.view.kick_dt*r.inverse[n]*(p.assembled.force[i]-contact.force[i]);
    const double ratio=std::abs(p.state.v[i]-omitted)/VelocityBudget();
    if(ratio>evidence.omitted_contact) { evidence.omitted_contact=ratio; evidence.omitted_contact_epoch=base.stamp.epoch; }
    const double wrong_v=base.v[i]+p.view.kick_dt*r.inverse[n]*premature.force[i];
    const double wrong_w=base.omega[i]+p.view.kick_dt*r.inverse_j[n]*premature.couple[i];
    evidence.early_endpoint=std::max(evidence.early_endpoint,std::abs(p.state.v[i]-wrong_v)/VelocityBudget());
    evidence.early_endpoint=std::max(evidence.early_endpoint,std::abs(p.state.omega[i]-wrong_w)/SpinBudget());
  }
}
} // namespace qeph_wall_test
