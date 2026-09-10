#include "MixedShellFeedbackFixture.h"

namespace mixed_feedback_test {
namespace {
double Component(tl::math::Vec3 v,unsigned a) { return a==0?v.x:a==1?v.y:v.z; }
void Ledger(long double actual,long double expected,long double terms) {
  const long double budget=256*std::numeric_limits<double>::epsilon()*terms+1e-12L*EnergyScale;
  ASSERT_TRUE(std::isfinite(actual)); ASSERT_TRUE(std::isfinite(expected));
  EXPECT_LE(std::abs(actual-expected),budget);
}
template<class Trial,class Diagnostic,std::size_t Count>
void FamilyWork(const std::array<std::size_t,Count>& nodes,const Trial& base,const Trial& next,
                const Diagnostic& d,const Snapshot& state,const Prepared& p) {
  long double kick=0,drift=0,terms=0;
  for(unsigned i=0;i<Count;++i) for(unsigned a=0;a<3;++a) {
    const auto j=3*nodes[i]+a;
    const long double force=-Component(base.internal_force[i],a),couple=-Component(base.internal_couple[i],a);
    const long double k=p.view.kick_dt*(force*(state.v[j]+p.endpoint.v[j])*.5L+
                                      couple*(state.omega[j]+p.endpoint.omega[j])*.5L);
    const long double w=force*(static_cast<long double>(p.endpoint.x[j])-state.x[j])+couple*H*p.endpoint.omega[j];
    kick+=k; drift+=w; terms+=std::abs(k)+std::abs(w);
  }
  Ledger(d.internal_kick_work,kick,terms); Ledger(d.internal_drift_work,drift,terms);
  for(unsigned i=0;i<2;++i) {
    const long double a=base.proposed_history.data().internal_work[i],b=next.proposed_history.data().internal_work[i];
    const long double increment=next.diagnostics.internal_work_increment[i];
    Ledger(d.internal_work[i],b,std::abs(b));
    Ledger(d.internal_work_increment[i],increment,std::abs(increment));
    Ledger(d.internal_work_increment[i],b-a,std::abs(a)+std::abs(b)+std::abs(increment));
  }
}
}
bool InitializeFeedback(Rig& r) {
  if(!r.PrepareReference())return false;
  const auto owner=r.initial.Initialize(r.owner);
  EXPECT_EQ(owner.status,fe::NodalStatus::Ok); if(owner.status!=fe::NodalStatus::Ok)return false;
  q::QephBatchConfig qc; qc.owner=r.owner.accepted(); qc.element_count=1;
  qc.configuration_id=FeedbackConfiguration; qc.qualification_id=FeedbackQualification; qc.usage=q::BatchUsage::CoupledForces;
  t::T3BatchConfig tc; tc.owner=r.owner.accepted(); tc.element_count=1;
  tc.configuration_id=FeedbackConfiguration; tc.qualification_id=FeedbackQualification; tc.usage=t::BatchUsage::CoupledForces;
  const auto qr=r.qeph.InitializeJoined(qc,r.binding);
  EXPECT_EQ(qr.status,q::BatchStatus::Success); if(qr.status!=q::BatchStatus::Success)return false;
  const auto tr=r.t3.InitializeJoined(tc,r.binding);
  EXPECT_EQ(tr.status,t::BatchStatus::Success); if(tr.status!=t::BatchStatus::Success)return false;
  return r.Bind();
}
bool PrepareFeedback(Rig& r,const Loads& load,Prepared& output,unsigned contributors) {
  if(contributors>3)return false;
  Prepared next; fe::NodalAssemblyView view; Staged cache;
  if(!Accepted(r,cache)||!temporal::BeginLoad(r.owner,load,next.token,view))return false;
  if(contributors&1) {
    const auto report=r.qeph.AssembleAccepted(view);
    EXPECT_EQ(report.status,q::BatchStatus::Success); if(report.status!=q::BatchStatus::Success)return false;
  }
  if(contributors&2) {
    const auto report=r.t3.AssembleAccepted(view);
    EXPECT_EQ(report.status,t::BatchStatus::Success); if(report.status!=t::BatchStatus::Success)return false;
  }
  const auto actual=Assembly(view); std::array<double,6*Nodes> expected{};
  for(unsigned n=0;n<Nodes;++n)for(unsigned a=0;a<3;++a) {
    expected[a*Nodes+n]=load.force[3*n+a]; expected[(a+3)*Nodes+n]=load.couple[3*n+a];
  }
  // Independent scalar negative cache scatter, in declared Q4 then T3 order.
  auto add=[&](const auto& nodes,const auto& result) {
    for(unsigned i=0;i<nodes.size();++i)for(unsigned a=0;a<3;++a) {
      expected[a*Nodes+nodes[i]]-=Component(result.internal_force[i],a);
      expected[(a+3)*Nodes+nodes[i]]-=Component(result.internal_couple[i],a);
    }
  };
  if(contributors&1)add(r.binding.qeph_nodes(),cache.qeph);
  if(contributors&2)add(r.binding.t3_nodes(),cache.t3);
  EXPECT_EQ(actual,expected);
  for(unsigned n=0;n<Nodes;++n)for(unsigned a=0;a<3;++a) {
    next.load.force[3*n+a]=actual[a*Nodes+n]; next.load.couple[3*n+a]=actual[(a+3)*Nodes+n];
  }
  auto report=r.owner.SealAssembly(next.token);
  EXPECT_EQ(report.status,fe::NodalStatus::Ok); if(report.status!=fe::NodalStatus::Ok)return false;
  report=fe::AdvanceStaggeredHistory(r.owner,next.token,
    {view.owner_id,view.accepted.base_epoch,view.attempt,H,1.,FeedbackQualification});
  EXPECT_EQ(report.status,fe::NodalStatus::Ok); if(report.status!=fe::NodalStatus::Ok)return false;
  report=r.owner.BorrowPrepared(next.token,&next.view);
  EXPECT_EQ(report.status,fe::NodalStatus::Ok);
  if(report.status!=fe::NodalStatus::Ok||!Endpoint(next.view,next.endpoint))return false;
  output=next; return !::testing::Test::HasFailure();
}
bool PublishFeedback(Rig& r,const Prepared& p,const Staged& next) {
  auto identity=[&](const auto& d) {
    EXPECT_TRUE(d.valid); EXPECT_TRUE(d.has_completed_interval); EXPECT_TRUE(d.accepted_force_assembled);
    EXPECT_FALSE(d.kinetic_available); EXPECT_EQ(d.kinetic_translation,0); EXPECT_EQ(d.kinetic_rotation,0);
    EXPECT_EQ(d.kinetic_physical_isotropic,0); EXPECT_EQ(d.kinetic_added_isotropic,0);
    EXPECT_EQ(d.owner_id,r.owner.accepted().owner_id); EXPECT_EQ(d.configuration_id,FeedbackConfiguration);
    EXPECT_EQ(d.qualification_id,FeedbackQualification); EXPECT_EQ(d.base_epoch,r.owner.accepted().epoch);
    EXPECT_EQ(d.attempt,p.view.attempt); EXPECT_EQ(d.epoch,d.base_epoch+1);
    EXPECT_EQ(d.time,p.view.proposed_time); EXPECT_EQ(d.velocity_time,p.view.velocity_time);
    EXPECT_EQ(d.kick_dt,d.base_epoch?H:H/2);
  };
  identity(next.diagnostics.qeph); identity(next.diagnostics.t3);
  EXPECT_EQ(next.diagnostics.qeph.usage,q::BatchUsage::CoupledForces);
  EXPECT_EQ(next.diagnostics.t3.usage,t::BatchUsage::CoupledForces);
  if(::testing::Test::HasFailure())return false;
  const auto report=r.publication.Commit(r.owner,p.token,next.diagnostics,Receipt(next));
  EXPECT_EQ(report.status,fe::ShellPublicationStatus::Success); return report.status==fe::ShellPublicationStatus::Success;
}
void CheckFeedback(const Rig& r,const Snapshot& base,const Staged& accepted,const Prepared& p,const Staged& next) {
  long double delta[2]{},work[2]{},terms[2]{},kinetic[4]{};
  const auto truth=shell_binding_test::Truth(r.input);
  for(unsigned n=0;n<Nodes;++n)for(unsigned a=0;a<3;++a) {
    const auto j=3*n+a;
    const long double old[]{base.v[j],base.omega[j]},now[]{p.endpoint.v[j],p.endpoint.omega[j]};
    const long double mass[]{r.binding.nodes()[n].native.mass,r.binding.nodes()[n].native.isotropic_inertia};
    const long double rhs[]{p.load.force[j],p.load.couple[j]};
    for(unsigned k=0;k<2;++k) {
      const long double expected=old[k]+p.view.kick_dt*rhs[k]/mass[k];
      EXPECT_LE(std::abs(now[k]-expected),temporal::ArithmeticTolerance*(1+std::abs(expected)));
      const auto before=.5L*mass[k]*old[k]*old[k],after=.5L*mass[k]*now[k]*now[k];
      const auto supplied=p.view.kick_dt*rhs[k]*(old[k]+now[k])*.5L;
      delta[k]+=after-before; work[k]+=supplied; terms[k]+=std::abs(before)+std::abs(after)+std::abs(supplied);
    }
    const long double x=base.x[j]+H*now[0];
    EXPECT_LE(std::abs(p.endpoint.x[j]-x),temporal::ArithmeticTolerance*(1+std::abs(x)));
    kinetic[0]+=.5L*truth[n].mass*now[0]*now[0]; kinetic[1]+=.5L*truth[n].total*now[1]*now[1];
    kinetic[2]+=.5L*truth[n].physical*now[1]*now[1]; kinetic[3]+=.5L*truth[n].added*now[1]*now[1];
  }
  for(unsigned i=0;i<2;++i)Ledger(delta[i],work[i],terms[i]);
  const auto& k=next.diagnostics.kinetic;
  const double measured[]{k.translation,k.rotation,k.physical_isotropic,k.added_isotropic};
  for(unsigned i=0;i<4;++i)Ledger(measured[i],kinetic[i],std::abs(kinetic[i]));
  FamilyWork(r.binding.qeph_nodes(),accepted.qeph,next.qeph,next.diagnostics.qeph,base,p);
  FamilyWork(r.binding.t3_nodes(),accepted.t3,next.t3,next.diagnostics.t3,base,p);
  const long double a=accepted.qeph.proposed_history.data().hourglass_viscous_work;
  const long double b=next.qeph.proposed_history.data().hourglass_viscous_work;
  Ledger(next.diagnostics.qeph.hourglass_viscous_work,b,std::abs(b));
  Ledger(next.diagnostics.qeph.hourglass_viscous_work_increment,b-a,std::abs(a)+std::abs(b));
}
void PreservedFeedback(Rig& r,const Snapshot& state,const Staged& elements,const fe::ShellBatchDiagnostics& common) {
  Snapshot now; ASSERT_TRUE(Read(r.owner,now)); SameState(state,now);
  Staged saved; ASSERT_TRUE(Accepted(r,saved)); ExactResults(elements,saved);
  EXPECT_EQ(Bytes(elements.diagnostics.qeph),Bytes(saved.diagnostics.qeph));
  EXPECT_EQ(Bytes(elements.diagnostics.t3),Bytes(saved.diagnostics.t3));
  fe::ShellBatchDiagnostics diagnostic;
  ASSERT_EQ(r.publication.CopyAcceptedDiagnostics(r.owner.accepted(),&diagnostic).status,fe::ShellPublicationStatus::Success);
  EXPECT_EQ(Bytes(diagnostic),Bytes(common));
}
} // namespace mixed_feedback_test
