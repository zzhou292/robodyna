#include "QephCoupledLedger.h"
#include <limits>

namespace qeph_coupled_test {
namespace {
constexpr long double Roundoff=256*std::numeric_limits<double>::epsilon();
using Wide3=std::array<long double,3>;
Wide3 Cross(const Wide3& a,const Wide3& b) {
  return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};
}
void Check(long double actual,long double expected,long double terms,long double floor,double& maximum) {
  const long double error=std::abs(actual-expected),budget=Roundoff*terms+1e-12L*floor;
  ASSERT_TRUE(std::isfinite(actual)); ASSERT_TRUE(std::isfinite(expected)); ASSERT_TRUE(std::isfinite(budget));
  EXPECT_LE(error,budget)<<std::setprecision(18)<<"actual="<<actual<<" expected="<<expected<<" budget="<<budget;
  maximum=std::max(maximum,static_cast<double>(budget?error/budget:(error?INFINITY:0)));
}
long double ExperimentEnergyScale(const Rig& r) {
  const auto amplitude=Amplitude(r);
  long double scale=0;
  for(unsigned i=0;i<3*r.n;++i)
    scale+=std::abs(amplitude.force[i])*Delta+std::abs(amplitude.couple[i])*Theta;
  return scale;
}
}
static void CheckLedgersWithScales(const Rig& r,const Snapshot& base,const Prepared& p,const Loads& load,
                  const PortResults& cache,const q::BatchDiagnostics& d,LedgerEvidence& evidence,const LedgerScales* scales) {
  Loads internal; const auto amplitude=Amplitude(r);
  const long double energy_scale=scales?scales->energy:ExperimentEnergyScale(r);
  long double linear_scale=0,angular_scale=0;
  if(scales) { linear_scale=scales->linear; angular_scale=scales->angular; }
  else for(unsigned i=0;i<3*r.n;++i) {
    linear_scale+=std::abs(amplitude.force[i])*4*H0;
    angular_scale+=(Side*std::abs(amplitude.force[i])+std::abs(amplitude.couple[i]))*4*H0;
  }
  long double internal_kick=0,internal_drift=0,internal_terms=0;
  for(unsigned e=0;e<r.count;++e) {
    for(unsigned i=0;i<4;++i) {
      const auto n=r.element[e].nodes[i]; const auto f=cache[e].internal_force[i],c=cache[e].internal_couple[i];
      const double force[]{f.x,f.y,f.z},couple[]{c.x,c.y,c.z};
      for(unsigned a=0;a<3;++a) {
        const auto j=3*n+a; internal.force[j]-=force[a]; internal.couple[j]-=couple[a];
        const long double kick=-p.view.kick_dt*(force[a]*(static_cast<long double>(base.v[j])+p.state.v[j])*.5L+
                                                  couple[a]*(static_cast<long double>(base.omega[j])+p.state.omega[j])*.5L);
        const long double drift=-force[a]*(static_cast<long double>(p.state.x[j])-base.x[j])-
                                 couple[a]*r.h*static_cast<long double>(p.state.omega[j]);
        internal_kick+=kick; internal_drift+=drift; internal_terms+=std::abs(kick)+std::abs(drift);
      }
    }
    if(base.stamp.epoch) {
      auto current=Interval(r,e,base,base.stamp.time,base.stamp.epoch);
      qeph_force_port_test::Balance(cache[e],current); // Existing independently frozen Q3c balance budget.
    }
  }
  long double kinetic[2]{},work[2]{},kinetic_terms[2]{},next_kinetic[4]{};
  Wide3 delta_p{},impulse{},p_terms{},delta_l{},torque{},l_terms{},rounding{};
  for(unsigned n=0;n<r.n;++n) {
    Wide3 x0{},x1{},p0{},p1{},force{},drift_error{};
    for(unsigned a=0;a<3;++a) {
      SCOPED_TRACE(n*3+a);
      const unsigned i=3*n+a;
      // Equality checks actual GPU assembly independently of native parity.
      EXPECT_EQ(p.assembled.force[i],internal.force[i]+load.force[i]);
      EXPECT_EQ(p.assembled.couple[i],internal.couple[i]+load.couple[i]);
      x0[a]=base.x[i]; x1[a]=p.state.x[i];
      p0[a]=r.mass[n]*static_cast<long double>(base.v[i]); p1[a]=r.mass[n]*static_cast<long double>(p.state.v[i]);
      force[a]=p.assembled.force[i];
      delta_p[a]+=p1[a]-p0[a]; impulse[a]+=p.view.kick_dt*force[a];
      p_terms[a]+=std::abs(p1[a])+std::abs(p0[a])+std::abs(p.view.kick_dt*force[a]);
      drift_error[a]=x1[a]-x0[a]-r.h*static_cast<long double>(p.state.v[i]);
      const double old[]{base.v[i],base.omega[i]},next[]{p.state.v[i],p.state.omega[i]};
      const double mass[]{r.mass[n],r.inertia[n]},rhs[]{p.assembled.force[i],p.assembled.couple[i]};
      for(unsigned k=0;k<2;++k) {
        const long double k0=.5L*mass[k]*old[k]*old[k],k1=.5L*mass[k]*next[k]*next[k];
        const long double w=p.view.kick_dt*rhs[k]*(static_cast<long double>(old[k])+next[k])*.5L;
        kinetic[k]+=k1-k0; work[k]+=w; kinetic_terms[k]+=std::abs(k0)+std::abs(k1)+std::abs(w);
        next_kinetic[k]+=k1;
      }
      next_kinetic[2]+=.5L*r.physical[n]*p.state.omega[i]*p.state.omega[i];
      next_kinetic[3]+=.5L*r.added[n]*p.state.omega[i]*p.state.omega[i];
      const long double old_spin=r.inertia[n]*static_cast<long double>(base.omega[i]);
      const long double next_spin=r.inertia[n]*static_cast<long double>(p.state.omega[i]);
      const long double spin_impulse=p.view.kick_dt*static_cast<long double>(p.assembled.couple[i]);
      delta_l[a]+=next_spin-old_spin; torque[a]+=spin_impulse;
      l_terms[a]+=std::abs(old_spin)+std::abs(next_spin)+std::abs(spin_impulse);
    }
    const auto old_orbit=Cross(x0,p0),next_orbit=Cross(x1,p1),arm_force=Cross(x0,force);
    const auto round=Cross(drift_error,p1);
    for(unsigned a=0;a<3;++a) {
      delta_l[a]+=next_orbit[a]-old_orbit[a]; torque[a]+=p.view.kick_dt*arm_force[a]; rounding[a]+=round[a];
      // Include the absolute cross-product operands, not only a cancelled sum.
      const unsigned b=(a+1)%3,c=(a+2)%3;
      l_terms[a]+=std::abs(x1[b]*p1[c])+std::abs(x1[c]*p1[b])+std::abs(x0[b]*p0[c])+std::abs(x0[c]*p0[b])+
        p.view.kick_dt*(std::abs(x0[b]*force[c])+std::abs(x0[c]*force[b]))+std::abs(round[a]);
    }
  }
  for(unsigned k=0;k<2;++k) { SCOPED_TRACE(k); Check(kinetic[k],work[k],kinetic_terms[k],energy_scale,evidence.kick_ratio); }
  for(unsigned a=0;a<3;++a) {
    SCOPED_TRACE(a);
    Check(delta_p[a],impulse[a],p_terms[a],linear_scale,evidence.momentum_ratio);
    Check(delta_l[a],torque[a]+rounding[a],l_terms[a],angular_scale,evidence.angular_ratio);
    evidence.angular_drift_rounding=std::max(evidence.angular_drift_rounding,static_cast<double>(std::abs(rounding[a])));
  }
  Check(d.internal_kick_work,internal_kick,internal_terms,energy_scale,evidence.internal_work_ratio);
  Check(d.internal_drift_work,internal_drift,internal_terms,energy_scale,evidence.internal_work_ratio);
  const double reported[]{d.kinetic_translation,d.kinetic_rotation,d.kinetic_physical_isotropic,d.kinetic_added_isotropic};
  for(unsigned k=0;k<4;++k) Check(reported[k],next_kinetic[k],std::abs(next_kinetic[k]),energy_scale,evidence.kick_ratio);
}
static void CheckSourceWorkWithScale(const Rig& r,const PortResults& accepted,const PortResults& candidate,
                     const q::BatchDiagnostics& d,LedgerEvidence& evidence,const long double* scale) {
  long double total[3]{},increment[3]{},difference[3]{};
  long double total_terms[3]{},increment_terms[3]{},difference_terms[3]{};
  for(unsigned e=0;e<r.count;++e) {
    const auto& old=accepted[e].proposed_history.data();
    const auto& next=candidate[e].proposed_history.data();
    const auto& diagnostic=candidate[e].diagnostics;
    const double base[]{old.internal_work[0],old.internal_work[1],old.hourglass_viscous_work};
    const double value[]{next.internal_work[0],next.internal_work[1],next.hourglass_viscous_work};
    const double delta[]{diagnostic.internal_work_increment[0],diagnostic.internal_work_increment[1],
                         diagnostic.hourglass_viscous_work_increment};
    for(unsigned c=0;c<3;++c) {
      total[c]+=value[c]; total_terms[c]+=std::abs(value[c]);
      increment[c]+=delta[c]; increment_terms[c]+=std::abs(delta[c]);
      // Subtract stored histories in long double as a separate interval oracle.
      // Absolute endpoints retain the rounding scale even after cancellation.
      difference[c]+=static_cast<long double>(value[c])-base[c];
      difference_terms[c]+=std::abs(static_cast<long double>(value[c]))+std::abs(static_cast<long double>(base[c]));
    }
  }
  const double reported[]{d.internal_work[0],d.internal_work[1],d.hourglass_viscous_work};
  const double reported_increment[]{d.internal_work_increment[0],d.internal_work_increment[1],
                                    d.hourglass_viscous_work_increment};
  const auto energy_scale=scale?*scale:ExperimentEnergyScale(r);
  for(unsigned c=0;c<3;++c) {
    SCOPED_TRACE(c);
    Check(reported[c],total[c],total_terms[c],energy_scale,evidence.source_work_ratio);
    Check(reported_increment[c],increment[c],increment_terms[c],energy_scale,evidence.source_work_ratio);
    Check(reported_increment[c],difference[c],difference_terms[c],energy_scale,evidence.source_work_ratio);
  }
  evidence.source_internal_work=static_cast<double>(total[0]+total[1]+total[2]);
}
void CheckLedgers(const Rig& r,const Snapshot& base,const Prepared& p,const Loads& load,
                  const PortResults& cache,const q::BatchDiagnostics& d,LedgerEvidence& evidence) {
  CheckLedgersWithScales(r,base,p,load,cache,d,evidence,nullptr);
}
void CheckLedgers(const Rig& r,const Snapshot& base,const Prepared& p,const Loads& load,
                  const PortResults& cache,const q::BatchDiagnostics& d,const LedgerScales& scales,LedgerEvidence& evidence) {
  ASSERT_TRUE(std::isfinite(scales.energy)&&scales.energy>0);
  ASSERT_TRUE(std::isfinite(scales.linear)&&scales.linear>0);
  ASSERT_TRUE(std::isfinite(scales.angular)&&scales.angular>0);
  CheckLedgersWithScales(r,base,p,load,cache,d,evidence,&scales);
}
void CheckSourceWork(const Rig& r,const PortResults& accepted,const PortResults& candidate,
                     const q::BatchDiagnostics& d,LedgerEvidence& evidence) {
  CheckSourceWorkWithScale(r,accepted,candidate,d,evidence,nullptr);
}
void CheckSourceWork(const Rig& r,const PortResults& accepted,const PortResults& candidate,
                     const q::BatchDiagnostics& d,long double scale,LedgerEvidence& evidence) {
  ASSERT_TRUE(std::isfinite(scale)&&scale>0);
  CheckSourceWorkWithScale(r,accepted,candidate,d,evidence,&scale);
}
double WrongKickSeparation(const Rig& r,const Snapshot& base,const Prepared& p,const Loads& load,bool initial) {
  double ratio=0;
  for(unsigned n=0;n<r.n;++n) for(unsigned a=0;a<3;++a) {
    const auto i=3*n+a; const double kick=initial?r.h:p.view.kick_dt;
    const double wrong_v=base.v[i]+kick*r.inverse[n]*load.force[i];
    const double wrong_w=base.omega[i]+kick*r.inverse_j[n]*load.couple[i];
    ratio=std::max(ratio,std::abs(p.state.v[i]-wrong_v)/VelocityBudget());
    ratio=std::max(ratio,std::abs(p.state.omega[i]-wrong_w)/SpinBudget());
  } return ratio;
}
} // namespace qeph_coupled_test
