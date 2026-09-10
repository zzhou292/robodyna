#include "ResidentCollectionFixture.h"
#include <iomanip>
#include <sstream>

namespace resident_collection_test {
namespace {
double Component(tl::math::Vec3 v,unsigned a) { return a==0?v.x:a==1?v.y:v.z; }
void Ledger(long double actual,long double expected,long double terms) {
  ASSERT_TRUE(std::isfinite(actual)); ASSERT_TRUE(std::isfinite(expected));
  const auto budget=256*std::numeric_limits<double>::epsilon()*terms+1e-12L*EnergyScale;
  EXPECT_LE(std::abs(actual-expected),budget);
}
template<class Results,class Getter,class Diagnostic>
void FamilyWork(const Results& base,const Results& next,Getter nodes,const Diagnostic& d,
                const Snapshot& state,const Prepared& p) {
  long double kick=0,drift=0,terms=0,work[2]{},increment[2]{},difference[2]{},work_terms[2]{};
  for(unsigned e=0;e<base.size();++e) {
    const auto& cell=nodes(e);
    for(unsigned local=0;local<cell.size();++local) for(unsigned a=0;a<3;++a) {
      const auto j=3*cell[local]+a;
      const long double force=-Component(base[e].internal_force[local],a);
      const long double couple=-Component(base[e].internal_couple[local],a);
      const long double k=p.view.kick_dt*(force*(state.v[j]+p.endpoint.v[j])*.5L+
                                        couple*(state.omega[j]+p.endpoint.omega[j])*.5L);
      const long double w=force*(static_cast<long double>(p.endpoint.x[j])-state.x[j])+couple*H*p.endpoint.omega[j];
      kick+=k; drift+=w; terms+=std::abs(k)+std::abs(w);
    }
    for(unsigned i=0;i<2;++i) {
      const long double a=base[e].proposed_history.data().internal_work[i];
      const long double b=next[e].proposed_history.data().internal_work[i];
      work[i]+=b; increment[i]+=next[e].diagnostics.internal_work_increment[i]; difference[i]+=b-a;
      work_terms[i]+=std::abs(a)+std::abs(b)+std::abs(next[e].diagnostics.internal_work_increment[i]);
    }
  }
  Ledger(d.internal_kick_work,kick,terms); Ledger(d.internal_drift_work,drift,terms);
  for(unsigned i=0;i<2;++i) {
    Ledger(d.internal_work[i],work[i],work_terms[i]);
    Ledger(d.internal_work_increment[i],increment[i],work_terms[i]);
    Ledger(d.internal_work_increment[i],difference[i],work_terms[i]);
  }
}
}
q::PrescribedInterval QInterval(const Rig& r,std::size_t e,const Prepared& p) {
  q::PrescribedInterval out; out.base_time=p.view.base_time; out.dt=H; out.sample_index=p.view.kinematics.base_epoch+1;
  for(unsigned i=0;i<4;++i) {
    const auto n=r.binding.qeph_nodes(e)[i];
    out.position_endpoint[i]={p.endpoint.x[3*n],p.endpoint.x[3*n+1],p.endpoint.x[3*n+2]};
    out.velocity_midpoint[i]={p.endpoint.v[3*n],p.endpoint.v[3*n+1],p.endpoint.v[3*n+2]};
    out.omega_midpoint[i]={p.endpoint.omega[3*n],p.endpoint.omega[3*n+1],p.endpoint.omega[3*n+2]};
  }
  return out;
}
t::PrescribedInterval TInterval(const Rig& r,std::size_t e,const Prepared& p) {
  t::PrescribedInterval out; out.base_time=p.view.base_time; out.dt=H; out.sample_index=p.view.kinematics.base_epoch+1;
  for(unsigned i=0;i<3;++i) {
    const auto n=r.binding.t3_nodes(e)[i];
    out.position[i]={p.endpoint.x[3*n],p.endpoint.x[3*n+1],p.endpoint.x[3*n+2]};
    out.velocity[i]={p.endpoint.v[3*n],p.endpoint.v[3*n+1],p.endpoint.v[3*n+2]};
    out.angular_velocity[i]={p.endpoint.omega[3*n],p.endpoint.omega[3*n+1],p.endpoint.omega[3*n+2]};
  }
  return out;
}
void CheckReference(const Rig& r) {
  std::array<shell_binding_test::WideMass,Capacity> mass{};
  for(unsigned e=0;e<QCount;++e) {
    const auto& cell=r.qinput[e];
    const long double area=.125L*.125L,m=1024.L*(1.L/32)*area/4;
    const long double physical=m*(1.L/32)*(1.L/32)/12,added=m*area/12;
    for(const auto node:cell.nodes) shell_binding_test::Add(mass[node],{m,physical+added,physical,added});
  }
  for(unsigned e=0;e<TCount;++e) {
    const auto& cell=r.tinput[e];
    const auto measured=shell_binding_test::independent::Independent(to::Native(cell.reference));
    for(unsigned local=0;local<3;++local) {
      const auto weight=measured.weight[local];
      shell_binding_test::Add(mass[cell.nodes[local]],
        {measured.mass*weight,measured.total*weight,measured.physical*weight,measured.added*weight});
    }
  }
  shell_binding_test::WideMass sum;
  for(unsigned n=0;n<Nodes;++n) {
    SCOPED_TRACE(n); const auto& actual=r.binding.nodes()[n].native;
    shell_binding_test::Near(actual.mass,mass[n].mass);
    shell_binding_test::Near(actual.isotropic_inertia,mass[n].total);
    shell_binding_test::Near(actual.physical_inertia,mass[n].physical);
    shell_binding_test::Near(actual.added_inertia,mass[n].added);
    shell_binding_test::Add(sum,mass[n]);
  }
  shell_binding_test::Near(r.binding.totals().mass,sum.mass);
  shell_binding_test::Near(r.binding.totals().isotropic_inertia,sum.total);
  EXPECT_EQ(r.binding.node_count(),Nodes); EXPECT_EQ(r.binding.qeph_count(),QCount); EXPECT_EQ(r.binding.t3_count(),TCount);
  EXPECT_EQ(r.binding.nodes()[116].position.x,1.5); EXPECT_EQ(r.binding.nodes()[116].position.y,1);
}
bool NativeSequence::Initialize(const Rig& r) {
  for(unsigned e=0;e<QCount;++e) {
    EXPECT_EQ(qn::Initialize(qeph_startup_test::NativeInput(r.binding.qeph_reference(e).input),qr[e]),qn::Status::kSuccess);
    EXPECT_EQ(qn::InitializeHistory(qr[e],{},qhistory[e]),qn::Status::kSuccess);
    qeph_startup_test::Agreement(r.binding.qeph_reference(e),qr[e].data());
  }
  for(unsigned e=0;e<TCount;++e) {
    EXPECT_EQ(tn::Initialize(to::Native(r.binding.t3_reference(e).input),tr[e]),tn::Status::kSuccess);
    EXPECT_EQ(tn::InitializeHistory(tr[e],{},thistory[e]),tn::Status::kSuccess);
    to::StartupAgreement(r.binding.t3_reference(e),tr[e]);
  }
  return !::testing::Test::HasFailure();
}
bool NativeSequence::Check(const Rig& r,const Prepared& p,const Staged& s) {
  for(unsigned e=0;e<QCount;++e) {
    SCOPED_TRACE(e); const auto interval=QInterval(r,e,p);
    EXPECT_EQ(qn::EvaluateForce(qr[e],qhistory[e],qeph_kinematics_test::NativeInterval(interval),qtrial[e]),qn::Status::kSuccess);
    qo::ForceAgreement(s.qeph[e],qtrial[e],r.binding.qeph_reference(e).input,interval);
    qo::Balance(s.qeph[e],interval);
  }
  for(unsigned e=0;e<TCount;++e) {
    SCOPED_TRACE(e); const auto interval=TInterval(r,e,p);
    EXPECT_EQ(tn::EvaluateForce(tr[e],thistory[e],to::Native(interval),ttrial[e]),tn::Status::kSuccess);
    to::Agreement(r.binding.t3_reference(e),interval,s.t3[e],ttrial[e]);
    to::oracle::Check(tr[e],thistory[e].data(),to::Native(interval),to::Native(tr[e],s.t3[e]));
  }
  return !::testing::Test::HasFailure();
}
void NativeSequence::Accept() {
  for(unsigned e=0;e<QCount;++e) qhistory[e]=qtrial[e].proposed_history;
  for(unsigned e=0;e<TCount;++e) thistory[e]=ttrial[e].proposed_history;
}
void CheckLedgers(const Rig& r,const Snapshot& base,const Staged& accepted,const Prepared& p,const Staged& next) {
  long double delta[2]{},supplied[2]{},terms[2]{},kinetic[4]{},base_kinetic[4]{};
  for(unsigned n=0;n<Nodes;++n) for(unsigned a=0;a<3;++a) {
    const auto j=3*n+a; const auto& mass=r.binding.nodes()[n].native;
    const long double m[]{mass.mass,mass.isotropic_inertia},old[]{base.v[j],base.omega[j]};
    const long double now[]{p.endpoint.v[j],p.endpoint.omega[j]},rhs[]{p.rhs[a*Nodes+n],p.rhs[(a+3)*Nodes+n]};
    for(unsigned k=0;k<2;++k) {
      const auto expected=old[k]+p.view.kick_dt*rhs[k]/m[k];
      EXPECT_LE(std::abs(now[k]-expected),temporal::ArithmeticTolerance*(1+std::abs(expected)));
      const auto before=.5L*m[k]*old[k]*old[k],after=.5L*m[k]*now[k]*now[k];
      const auto work=p.view.kick_dt*rhs[k]*(old[k]+now[k])*.5L;
      delta[k]+=after-before; supplied[k]+=work; terms[k]+=std::abs(before)+std::abs(after)+std::abs(work);
    }
    const long double x=base.x[j]+H*now[0];
    EXPECT_LE(std::abs(p.endpoint.x[j]-x),temporal::ArithmeticTolerance*(1+std::abs(x)));
    const long double weights[]{mass.mass,mass.isotropic_inertia,mass.physical_inertia,mass.added_inertia};
    for(unsigned k=0;k<4;++k) {
      const auto field=k?1:0; kinetic[k]+=.5L*weights[k]*now[field]*now[field];
      base_kinetic[k]+=.5L*weights[k]*old[field]*old[field];
    }
  }
  for(unsigned i=0;i<2;++i) Ledger(delta[i],supplied[i],terms[i]);
  const auto& k=next.diagnostics.kinetic; const auto& b=next.diagnostics.base_kinetic;
  const double actual[]{k.translation,k.rotation,k.physical_isotropic,k.added_isotropic};
  const double old[]{b.translation,b.rotation,b.physical_isotropic,b.added_isotropic};
  for(unsigned i=0;i<4;++i) {
    Ledger(actual[i],kinetic[i],std::abs(kinetic[i])); Ledger(old[i],base_kinetic[i],std::abs(base_kinetic[i]));
  }
  FamilyWork(accepted.qeph,next.qeph,[&](unsigned e)->const auto& { return r.binding.qeph_nodes(e); },next.diagnostics.qeph,base,p);
  FamilyWork(accepted.t3,next.t3,[&](unsigned e)->const auto& { return r.binding.t3_nodes(e); },next.diagnostics.t3,base,p);
  long double work=0,increment=0,difference=0,work_terms=0;
  for(unsigned e=0;e<QCount;++e) {
    const long double a=accepted.qeph[e].proposed_history.data().hourglass_viscous_work;
    const long double b=next.qeph[e].proposed_history.data().hourglass_viscous_work;
    work+=b; increment+=next.qeph[e].diagnostics.hourglass_viscous_work_increment; difference+=b-a;
    work_terms+=std::abs(a)+std::abs(b)+std::abs(next.qeph[e].diagnostics.hourglass_viscous_work_increment);
  }
  Ledger(next.diagnostics.qeph.hourglass_viscous_work,work,work_terms);
  Ledger(next.diagnostics.qeph.hourglass_viscous_work_increment,increment,work_terms);
  Ledger(next.diagnostics.qeph.hourglass_viscous_work_increment,difference,work_terms);
}
void SameState(const Snapshot& a,const Snapshot& b,bool same_owner) {
  EXPECT_EQ(a.x,b.x); EXPECT_EQ(a.v,b.v); EXPECT_EQ(a.orientation,b.orientation); EXPECT_EQ(a.omega,b.omega);
  EXPECT_EQ(a.reaction,b.reaction); EXPECT_EQ(a.couple,b.couple);
  auto stamp=a.stamp; if(!same_owner) stamp.owner_id=b.stamp.owner_id;
  temporal::SameStamp(stamp,b.stamp);
}
void ExactResults(const Staged& a,const Staged& b) {
  for(unsigned e=0;e<QCount;++e) EXPECT_EQ(Bytes(a.qeph[e]),Bytes(b.qeph[e]));
  for(unsigned e=0;e<TCount;++e) to::Exact(a.t3[e],b.t3[e]);
  EXPECT_EQ(Bytes(a.diagnostics.kinetic),Bytes(b.diagnostics.kinetic));
}
void Property(const char* name,double value) {
  std::ostringstream out; out<<std::setprecision(std::numeric_limits<double>::max_digits10)<<value;
  ::testing::Test::RecordProperty(name,out.str());
}
} // namespace resident_collection_test
