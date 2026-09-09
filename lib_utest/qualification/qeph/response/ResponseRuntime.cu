#include "ResponseRuntime.h"
#include "ResponseSamples.h"
#include "../coupled/QephCoupledLedger.h"
#include <chrono>
#include <iostream>

namespace tl::qualification::qeph::response {
namespace c=qeph_coupled_test;
namespace {
static_assert(H0==c::H0&&Side==c::Side&&Theta==c::Theta&&Thickness==c::Thickness);
struct Scope {
  Run& run; c::Rig& rig;
  std::chrono::steady_clock::time_point started=std::chrono::steady_clock::now();
  ~Scope() {
    rig.owner.Discard(); rig.batch.DiscardTrial();
    run.elapsed_seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
    std::size_t total=0; if(cudaMemGetInfo(&run.cuda_free_after_run,&total)!=cudaSuccess) run.cuda_free_after_run=0;
    if(!run.completed) std::cerr<<"Response stopped before publication: "<<run.failure<<"; accepted epoch "<<run.accepted_steps<<'\n';
  }
};
State StateOf(const c::Snapshot& s) {
  State out;
  std::copy_n(s.x.begin(),out.x.size(),out.x.begin()); std::copy_n(s.v.begin(),out.v.size(),out.v.begin());
  std::copy_n(s.omega.begin(),out.omega.size(),out.omega.begin()); std::copy_n(s.q.begin(),out.q.size(),out.q.begin()); return out;
}
Results ResultsOf(const c::PortResults& r) { return {r[0],r[1]}; }
c::Loads Load(const c::Rig& r,double time) {
  auto load=c::Amplitude(r); const double factor=PulseFactor(time);
  for(unsigned i=0;i<3*r.n;++i) { load.force[i]*=factor; load.couple[i]*=factor; } return load;
}
void CheckIdentity(const c::Rig& r,const c::Prepared& p,const port::BatchDiagnostics& d) {
  EXPECT_TRUE(d.valid); EXPECT_TRUE(d.has_completed_interval); EXPECT_TRUE(d.accepted_force_assembled);
  EXPECT_EQ(d.usage,port::BatchUsage::CoupledForces); EXPECT_EQ(d.phase,port::BatchPhase::Prepared);
  EXPECT_EQ(d.qualification_id,Qualification); EXPECT_EQ(d.configuration_id,Configuration);
  EXPECT_EQ(d.owner_id,r.owner.accepted().owner_id); EXPECT_EQ(d.base_epoch,r.owner.accepted().epoch);
  EXPECT_EQ(d.epoch,d.base_epoch+1); EXPECT_EQ(d.attempt,p.view.attempt);
  EXPECT_EQ(d.base_time,r.owner.accepted().time); EXPECT_EQ(d.time,p.view.proposed_time);
  EXPECT_EQ(d.velocity_time,p.view.velocity_time); EXPECT_EQ(d.base_velocity_time,p.view.base_velocity_time);
  EXPECT_EQ(d.kick_dt,p.view.kick_dt); EXPECT_EQ(d.kick_dt,d.base_epoch?r.h:.5*r.h);
}
long double ExternalIncrement(const c::Rig& r,const c::Snapshot& base,const c::Prepared& p,const c::Loads& load) {
  long double work=0;
  for(unsigned i=0;i<3*r.n;++i) work+=load.force[i]*(static_cast<long double>(p.state.x[i])-base.x[i])+
      load.couple[i]*r.h*static_cast<long double>(p.state.omega[i]);
  return work;
}
void SaveEvidence(Run& run,const c::LedgerEvidence& e) {
  run.ledger_maxima={e.kick_ratio,e.momentum_ratio,e.angular_ratio,e.internal_work_ratio,e.source_work_ratio,
                    e.angular_drift_rounding,e.source_internal_work};
}
}
void Execute(Run& run) {
  ASSERT_TRUE(ValidConfig(run.config)); ASSERT_TRUE(run.samples.empty());
  run.failure="CUDA initialization";
  int count=0; ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess); ASSERT_GT(count,0);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  std::size_t total=0; ASSERT_EQ(cudaMemGetInfo(&run.cuda_free_before,&total),cudaSuccess);
  c::Rig r(run.config.cells); Scope scope{run,r};
  run.failure="Reference/owner/batch initialization";
  ASSERT_TRUE(c::InitializeCoupled(r,H0/run.config.refinement,Qualification,Configuration));
  for(unsigned n=0;n<r.n;++n) {
    EXPECT_EQ(r.mass[n],run.model.mass[n]); EXPECT_EQ(r.inertia[n],run.model.inertia[n]);
    EXPECT_EQ(r.physical[n],run.model.physical[n]); EXPECT_EQ(r.added[n],run.model.added[n]);
  }
  const auto allocation=r.batch.allocations(),owner_allocation=r.owner.allocations();
  run.batch_device_bytes=allocation.device_bytes; run.batch_allocations=allocation.device_allocations;
  run.owner_device_bytes=owner_allocation.device_bytes; run.owner_allocations=owner_allocation.device_allocations;
  ASSERT_EQ(cudaMemGetInfo(&run.cuda_free_after_initialize,&total),cudaSuccess);
  run.owner_id=r.owner.accepted().owner_id;
  c::NativeSequence native; ASSERT_TRUE(native.Initialize(r));
  c::Snapshot base; ASSERT_TRUE(c::Read(r.owner,base));
  c::PortResults cache{}; port::BatchDiagnostics accepted; ASSERT_TRUE(c::ReadAccepted(r,cache,accepted));
  ASSERT_EQ(accepted.qualification_id,Qualification); ASSERT_EQ(accepted.configuration_id,Configuration);
  ASSERT_FALSE(accepted.has_completed_interval);
  Limits initial_limits; std::string error;
  ASSERT_TRUE(Observe(run.model,r.h,0,StateOf(base),ResultsOf(cache),0,run.last_accepted,initial_limits,error))<<error;
  run.samples.push_back(run.last_accepted);
  c::LedgerEvidence evidence;
  long double external_work=0,linear_impulse[3]{},angular_impulse[3]{};
  auto progress=std::chrono::steady_clock::now();
  const unsigned steps=4096*run.config.refinement,stride=16*run.config.refinement;
  for(unsigned step=0;step<steps;++step) {
    run.failure="Accepted-state/native proposal"; run.attempted_steps=step+1;
    ASSERT_TRUE(c::Read(r.owner,base)); c::OwnerAgreement(r,base,native.state);
    EXPECT_EQ(base.stamp.epoch,native.epoch); EXPECT_EQ(base.stamp.time,native.time);
    const auto load=Load(r,native.time); c::NativeProposal expected;
    ASSERT_TRUE(native.Propose(r,load,expected));
    run.failure="Cached assembly/owner preparation";
    c::Prepared prepared; ASSERT_TRUE(c::PrepareCoupled(r,load,prepared,Qualification));
    c::OwnerAgreement(r,prepared.state,expected.state);
    run.failure="Candidate force/history and native parity";
    c::PortResults next{}; port::BatchDiagnostics d;
    ASSERT_TRUE(c::Candidate(r,prepared.view,d,next)); CheckIdentity(r,prepared,d);
    for(unsigned e=0;e<r.count;++e) qeph_force_port_test::ForceAgreement(next[e],expected.cache[e],r.element[e].reference.input,
        c::Interval(r,e,prepared.state,native.time,native.epoch));
    c::CheckLedgers(r,base,prepared,load,cache,d,evidence); c::CheckSourceWork(r,cache,next,d,evidence); SaveEvidence(run,evidence);
    run.failure="Endpoint observation/small-response domain";
    const long double proposed_work=external_work+ExternalIncrement(r,base,prepared,load);
    Sample staged; Limits observed;
    ASSERT_TRUE(Observe(run.model,r.h,step+1,StateOf(prepared.state),ResultsOf(next),proposed_work,staged,observed,error))<<error;
    EXPECT_TRUE(InsideLimits(observed));
    long double proposed_linear[3]{linear_impulse[0],linear_impulse[1],linear_impulse[2]};
    long double proposed_angular[3]{angular_impulse[0],angular_impulse[1],angular_impulse[2]};
    for(unsigned n=0;n<r.n;++n) for(unsigned a=0;a<3;++a) {
      const unsigned b=(a+1)%3,c=(a+2)%3;
      proposed_linear[a]+=prepared.view.kick_dt*static_cast<long double>(load.force[3*n+a]);
      proposed_angular[a]+=prepared.view.kick_dt*(static_cast<long double>(load.couple[3*n+a])+
        base.x[3*n+b]*static_cast<long double>(load.force[3*n+c])-base.x[3*n+c]*static_cast<long double>(load.force[3*n+b]));
    }
    for(unsigned a=0;a<3;++a) { EXPECT_TRUE(std::isfinite(static_cast<double>(proposed_linear[a]))); EXPECT_TRUE(std::isfinite(static_cast<double>(proposed_angular[a]))); }
    EXPECT_EQ(r.batch.allocations().device_bytes,allocation.device_bytes);
    EXPECT_EQ(r.batch.allocations().device_allocations,allocation.device_allocations);
    EXPECT_EQ(r.owner.allocations().device_bytes,owner_allocation.device_bytes);
    EXPECT_EQ(r.owner.allocations().device_allocations,owner_allocation.device_allocations);
    // All native/ledger/observer operations precede this receipt. No assertions
    // or fallible output readback can be deferred into joint publication.
    ASSERT_FALSE(::testing::Test::HasFailure())<<"No receipt after a failed scientific check";
    run.failure="Joint owner/history/cache publication";
    ASSERT_TRUE(c::Commit(r,prepared.token,d));
    native.Accept(r,expected); cache=next; external_work=proposed_work;
    run.last_accepted=staged; run.accepted_steps=step+1;
    AccumulateLimits(run.observed,observed);
    if(std::abs(staged.residual)>run.maximum_abs_residual) { run.maximum_abs_residual=std::abs(staged.residual); run.residual_time=staged.time; }
    if(staged.time==Pulse) run.external_work_at_pulse=staged.external_work;
    for(unsigned a=0;a<3;++a) { linear_impulse[a]=proposed_linear[a]; angular_impulse[a]=proposed_angular[a];
      run.external_linear_impulse[a]=static_cast<double>(linear_impulse[a]); run.external_angular_impulse[a]=static_cast<double>(angular_impulse[a]); }
    if((step+1)%stride==0) {
      run.samples.push_back(staged); // Capacity was reserved before initialization.
      const auto maximum=ResponseMaximum(run.fields,run.samples.front(),staged);
      if(maximum.value>run.maximum_response.value) run.maximum_response=maximum;
    }
    const auto now=std::chrono::steady_clock::now();
    if(now-progress>std::chrono::seconds(30)) {
      std::cout<<"Accepted "<<run.accepted_steps<<'/'<<steps<<" endpoints; time "<<staged.time<<" s\n"<<std::flush; progress=now;
    }
  }
  run.failure="Completed-run structural validation";
  EXPECT_EQ(run.samples.size(),SampleCount); EXPECT_EQ(run.last_accepted.time,Horizon);
  ASSERT_FALSE(::testing::Test::HasFailure()); run.completed=true; run.failure.clear();
}
} // namespace tl::qualification::qeph::response
