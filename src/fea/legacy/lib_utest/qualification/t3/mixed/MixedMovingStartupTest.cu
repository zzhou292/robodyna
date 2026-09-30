#include "MixedMovingStartupFixture.h"
#include <cmath>
#include <limits>

namespace mixed_moving_test {
using S=fe::ShellPublicationStatus;
TEST_F(MixedShellCuda, MovingCommonKineticAndFirstHalfKickMatchNativeWithoutDuplicateEnergy) {
  for(bool accelerated:{false,true}) {
    SCOPED_TRACE(accelerated);
    Rig r; ASSERT_TRUE(InitializeMoving(r));
    NativePair native; ASSERT_TRUE(native.Initialize(r));
    Staged cache; ASSERT_TRUE(Accepted(r,cache));
    fe::ShellBatchDiagnostics initial;
    ASSERT_EQ(r.publication.CopyAcceptedDiagnostics(r.owner.accepted(),&initial).status,S::Success);
    ASSERT_NO_FATAL_FAILURE(InitialKinetic(r,cache,initial));
    const auto allocations=r.publication.allocations();
    Loads load;
    if(accelerated) for(unsigned n=0;n<Nodes;++n) load.force[3*n]=r.binding.nodes()[n].native.mass;
    for(unsigned step=0;step<2;++step) {
      SCOPED_TRACE(step);
      Snapshot base; ASSERT_TRUE(Read(r.owner,base));
      Prepared p; ASSERT_TRUE(PrepareFeedback(r,load,p));
      Staged next; ASSERT_TRUE(Evaluate(r,p,next,step%2));
      NativeTrials expected; ASSERT_TRUE(native.Check(r,p,next,expected));
      ASSERT_NO_FATAL_FAILURE(CheckFeedback(r,base,cache,p,next));
      EXPECT_EQ(p.view.kick_dt,step?H:H/2);
      if(!step) EXPECT_EQ(next.diagnostics.base_kinetic.translation,initial.kinetic.translation);
      for(unsigned n=0;n<Nodes;++n) {
        const double time=(step+1)*H;
        EXPECT_NEAR(p.endpoint.v[3*n],Velocity.x+(accelerated?(step+.5)*H:0),2e-13*(1+Velocity.x));
        EXPECT_NEAR(p.endpoint.x[3*n],r.initial.x[3*n]+Velocity.x*time+(accelerated?.5*time*time:0),2e-13);
        EXPECT_NEAR(p.endpoint.v[3*n+1],Velocity.y,2e-13);
        EXPECT_NEAR(p.endpoint.v[3*n+2],Velocity.z,2e-13);
      }
      ASSERT_TRUE(PublishFeedback(r,p,next)); native.Accept(expected);
      ASSERT_TRUE(Accepted(r,cache));
    }
    EXPECT_EQ(r.publication.allocations().device_bytes,allocations.device_bytes);
    EXPECT_EQ(r.publication.allocations().device_allocations,1u);
  }
  RecordProperty("native_qeph_intervals",4); RecordProperty("native_t3_intervals",4);
}

TEST_F(MixedShellCuda, MovingFirstTrialRejectionPreservesCommonK0AndExactRetry) {
  Rig r,clean; ASSERT_TRUE(InitializeMoving(r)); ASSERT_TRUE(InitializeMoving(clean));
  Snapshot before; ASSERT_TRUE(Read(r.owner,before));
  Staged cache; ASSERT_TRUE(Accepted(r,cache));
  fe::ShellBatchDiagnostics common;
  ASSERT_EQ(r.publication.CopyAcceptedDiagnostics(r.owner.accepted(),&common).status,S::Success);
  Prepared rejected; ASSERT_TRUE(PrepareFeedback(r,{},rejected));
  Staged candidate; ASSERT_TRUE(Evaluate(r,rejected,candidate));
  auto receipt=Receipt(candidate); receipt.passed=false;
  EXPECT_EQ(r.publication.Commit(r.owner,rejected.token,candidate.diagnostics,receipt).status,S::StaleTrial);
  ASSERT_NO_FATAL_FAILURE(PreservedFeedback(r,before,cache,common));
  Prepared p,reference; Staged next,expected;
  ASSERT_TRUE(PrepareFeedback(r,{},p)); ASSERT_TRUE(Evaluate(r,p,next,true));
  ASSERT_TRUE(PrepareFeedback(clean,{},reference)); ASSERT_TRUE(Evaluate(clean,reference,expected));
  EXPECT_EQ(p.endpoint.x,reference.endpoint.x); EXPECT_EQ(p.endpoint.v,reference.endpoint.v);
  EXPECT_EQ(p.endpoint.q,reference.endpoint.q); EXPECT_EQ(p.endpoint.omega,reference.endpoint.omega);
  ExactResults(next,expected);
  ASSERT_TRUE(PublishFeedback(r,p,next)); ASSERT_TRUE(PublishFeedback(clean,reference,expected));
  EXPECT_EQ(r.owner.accepted().epoch,1u); EXPECT_EQ(clean.owner.accepted().epoch,1u);
}

TEST_F(MixedShellCuda, MovingStartupRequiresAuthenticSourcesAndMatchingDeclarations) {
  Rig r; ASSERT_TRUE(MovingReference(r));
  ASSERT_EQ(r.initial.Initialize(r.owner).status,fe::NodalStatus::Ok); ASSERT_TRUE(Participants(r));
  for(unsigned family=0;family<2;++family) {
    SCOPED_TRACE(family);
    fe::NodalTrialToken token; fe::NodalAssemblyView view;
    ASSERT_EQ(r.owner.BeginTrial(&token,&view).status,fe::NodalStatus::Ok);
    if(!family) EXPECT_EQ(r.qeph.AssembleAccepted(view).status,q::BatchStatus::InvalidInput);
    else EXPECT_EQ(r.t3.AssembleAccepted(view).status,t::BatchStatus::InvalidInput);
    EXPECT_EQ(r.owner.SealAssembly(token).status,fe::NodalStatus::ContributorFailure); r.Discard();
  }
  Rig foreign; ASSERT_TRUE(MovingReference(foreign));
  ASSERT_EQ(foreign.initial.Initialize(foreign.owner).status,fe::NodalStatus::Ok);
  for(unsigned family=0;family<2;++family) {
    SCOPED_TRACE(family);
    fe::NodalTrialToken token,other_token; fe::NodalAssemblyView view,other;
    ASSERT_EQ(r.owner.BeginTrial(&token,&view).status,fe::NodalStatus::Ok);
    ASSERT_EQ(foreign.owner.BeginTrial(&other_token,&other).status,fe::NodalStatus::Ok);
    view.accepted=other.accepted; // Owner id/metadata still claim r; physical pointers are foreign.
    if(!family) EXPECT_EQ(r.qeph.AssembleAccepted(r.owner,view).status,q::BatchStatus::StaleTrial);
    else EXPECT_EQ(r.t3.AssembleAccepted(r.owner,view).status,t::BatchStatus::StaleTrial);
    r.Discard(); foreign.Discard();
  }
  EXPECT_EQ(r.publication.allocations().device_allocations,0u);
  ASSERT_TRUE(BindMoving(r)); // Rejected authentication did not consume startup binding.

  // At zero common speed both declarations can bind actual rest fields, but
  // rest and explicitly moving are distinct immutable startup scopes.
  Rig mismatch; ASSERT_TRUE(mismatch.PrepareReference());
  ASSERT_EQ(mismatch.initial.Initialize(mismatch.owner).status,fe::NodalStatus::Ok);
  auto qc=QConfig(mismatch); qc.startup.uniform_velocity={};
  auto tc=TConfig(mismatch); tc.startup={};
  ASSERT_EQ(mismatch.qeph.InitializeJoined(qc,mismatch.binding).status,q::BatchStatus::Success);
  ASSERT_EQ(mismatch.t3.InitializeJoined(tc,mismatch.binding).status,t::BatchStatus::Success);
  fe::NodalTrialToken token; fe::NodalAssemblyView view;
  ASSERT_EQ(mismatch.owner.BeginTrial(&token,&view).status,fe::NodalStatus::Ok);
  ASSERT_EQ(mismatch.qeph.AssembleAccepted(mismatch.owner,view).status,q::BatchStatus::Success);
  ASSERT_EQ(mismatch.t3.AssembleAccepted(mismatch.owner,view).status,t::BatchStatus::Success);
  mismatch.Discard();
  EXPECT_EQ(mismatch.publication.Initialize(mismatch.owner,mismatch.qeph,mismatch.t3).status,S::InvalidInput);
  EXPECT_EQ(mismatch.publication.allocations().device_allocations,0u);
  fe::ShellBatchStartup a=MovingStartup(),b=a;
  b.uniform_velocity.z=std::nextafter(b.uniform_velocity.z,1.);
  EXPECT_FALSE(fe::shell_startup_detail::SameStartup(a,b));
  b=a; a.uniform_velocity.z=0.; b.uniform_velocity.z=-0.;
  EXPECT_FALSE(fe::shell_startup_detail::SameStartup(a,b));
  // Rest historically accepts either represented zero sign; preserve that
  // behavior while explicit moving declarations retain exact velocity bits.
  Rig legacy; ASSERT_TRUE(legacy.PrepareReference());
  ASSERT_EQ(legacy.initial.Initialize(legacy.owner).status,fe::NodalStatus::Ok);
  auto legacy_q=QConfig(legacy); legacy_q.startup={fe::ShellBatchStartupKind::ReferenceRest,{-0.,0.,-0.}};
  auto legacy_t=TConfig(legacy); legacy_t.startup={};
  ASSERT_EQ(legacy.qeph.InitializeJoined(legacy_q,legacy.binding).status,q::BatchStatus::Success);
  ASSERT_EQ(legacy.t3.InitializeJoined(legacy_t,legacy.binding).status,t::BatchStatus::Success);
  ASSERT_TRUE(legacy.Bind());
}

TEST_F(MixedShellCuda, MovingLateUnionNodeFaultsLeaveBothFamiliesAndCommonKineticUnpublished) {
  for(unsigned fault=0;fault<7;++fault) {
    SCOPED_TRACE(fault);
    Rig r; ASSERT_TRUE(MovingReference(r)); const auto n=Nodes-1,j=3*n;
    if(fault==0) r.initial.v[j]=std::nextafter(Velocity.x,9.);
    if(fault==1) r.initial.omega[j]=.125;
    if(fault==2) { r.initial.q[4*n]=0; r.initial.q[4*n+1]=1; }
    if(fault==3) r.initial.inverse[n]*=2;
    if(fault==4) r.initial.inverse_inertia[n]*=2;
    if(fault==5) r.initial.x[j]+=.125;
    if(fault==6) r.initial.v[j+2]=std::nextafter(Velocity.z,1.);
    ASSERT_EQ(r.initial.Initialize(r.owner).status,fe::NodalStatus::Ok); ASSERT_TRUE(Participants(r));
    Snapshot before; ASSERT_TRUE(Read(r.owner,before));
    for(unsigned family=0;family<2;++family) {
      SCOPED_TRACE(family);
      fe::NodalTrialToken token; fe::NodalAssemblyView view;
      ASSERT_EQ(r.owner.BeginTrial(&token,&view).status,fe::NodalStatus::Ok);
      if(!family) {
        const auto report=r.qeph.AssembleAccepted(r.owner,view);
        EXPECT_EQ(report.status,fault==3||fault==4?q::BatchStatus::InvalidMass:q::BatchStatus::InvalidInput);
        EXPECT_EQ(report.node,n);
      } else {
        const auto report=r.t3.AssembleAccepted(r.owner,view);
        EXPECT_EQ(report.status,fault==3||fault==4?t::BatchStatus::InvalidMass:t::BatchStatus::InvalidInput);
        EXPECT_EQ(report.node,n);
      }
      EXPECT_EQ(r.owner.SealAssembly(token).status,fe::NodalStatus::ContributorFailure); r.Discard();
    }
    EXPECT_EQ(r.publication.Initialize(r.owner,r.qeph,r.t3).status,S::InvalidInput);
    EXPECT_EQ(r.publication.allocations().device_allocations,0u);
    q::ForceTrial qr; t::ForceTrial tr; q::BatchDiagnostics qd; t::BatchDiagnostics td;
    const auto qb=Bytes(qr); const auto tb=Bytes(tr);
    const auto qdb=Bytes(qd); const auto tdb=Bytes(td);
    EXPECT_EQ(r.qeph.CopyAcceptedResults(r.owner.accepted(),&qr,1,&qd).status,q::BatchStatus::NotBound);
    EXPECT_EQ(r.t3.CopyAcceptedResults(r.owner.accepted(),&tr,1,&td).status,t::BatchStatus::NotBound);
    EXPECT_EQ(Bytes(qr),qb); EXPECT_EQ(Bytes(tr),tb); EXPECT_EQ(Bytes(qd),qdb); EXPECT_EQ(Bytes(td),tdb);
    Snapshot after; ASSERT_TRUE(Read(r.owner,after)); SameState(before,after);
  }
}
} // namespace mixed_moving_test
