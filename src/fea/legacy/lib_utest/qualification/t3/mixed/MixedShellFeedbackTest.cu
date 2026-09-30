#include "MixedShellFeedbackFixture.h"

namespace mixed_feedback_test {
namespace {
using S=fe::ShellPublicationStatus;
static __global__ void SetPosition(double* x,unsigned node,double a,double b,double c) {
  x[3*node]=a; x[3*node+1]=b; x[3*node+2]=c;
}
bool Prime(Rig& r,NativePair& native) {
  Snapshot base; Staged accepted,next; Prepared p; NativeTrials truth;
  if(!Read(r.owner,base)||!Accepted(r,accepted)||!PrepareFeedback(r,Schedule(r,0),p)||
     !Evaluate(r,p,next)||!native.Check(r,p,next,truth))return false;
  CheckFeedback(r,base,accepted,p,next);
  if(!PublishFeedback(r,p,next))return false;
  native.Accept(truth); return true;
}
}
TEST_F(MixedShellCuda, CoupledPulseThenFreeStepsConsumeBothNonzeroNativeCaches) {
  Rig r; ASSERT_TRUE(InitializeFeedback(r));
  NativePair native; ASSERT_TRUE(native.Initialize(r));
  const std::array<fe::NodalAllocationInfo,4> allocation{
    r.owner.allocations(),r.qeph.allocations(),r.t3.allocations(),r.publication.allocations()};
  Staged accepted; ASSERT_TRUE(Accepted(r,accepted));
  double qforce=0,tforce=0,qwork=0,twork=0;
  for(unsigned step=0;step<4;++step) {
    SCOPED_TRACE(step);
    Snapshot base; ASSERT_TRUE(Read(r.owner,base));
    const Loads pulse=step?Loads{}:Schedule(r,0);
    Prepared p; ASSERT_TRUE(PrepareFeedback(r,pulse,p));
    Staged next; ASSERT_TRUE(Evaluate(r,p,next,step%2));
    NativeTrials truth; ASSERT_TRUE(native.Check(r,p,next,truth));
    CheckFeedback(r,base,accepted,p,next);
    if(step==1) {
      // Distinct physical nodes exclude cancellation between families. Each
      // omitted family would change its node's measured next velocity by far
      // more than the existing owner arithmetic budget.
      double missing_q=0,missing_t=0;
      const auto fq=accepted.qeph.internal_force[0],ft=accepted.t3.internal_force[1];
      const double qcomponent[]{fq.x,fq.y,fq.z},tcomponent[]{ft.x,ft.y,ft.z};
      for(unsigned a=0;a<3;++a) {
        missing_q=std::max(missing_q,std::abs(H*qcomponent[a]/r.binding.nodes()[0].native.mass));
        missing_t=std::max(missing_t,std::abs(H*tcomponent[a]/r.binding.nodes()[4].native.mass));
      }
      EXPECT_GT(missing_q,32*temporal::ArithmeticTolerance);
      EXPECT_GT(missing_t,32*temporal::ArithmeticTolerance);
      EXPECT_NE(p.endpoint.v,base.v);
      EXPECT_NE(next.diagnostics.qeph.internal_kick_work,0);
      EXPECT_NE(next.diagnostics.t3.internal_kick_work,0);
      Property("omitted_q4_velocity_change_m_s",missing_q); Property("omitted_t3_velocity_change_m_s",missing_t);
    }
    for(auto f:next.qeph.internal_force)qforce=std::max(qforce,qeph_startup_test::Length(f));
    for(auto f:next.t3.internal_force)tforce=std::max(tforce,qeph_startup_test::Length(f));
    qwork=std::max(qwork,std::abs(next.diagnostics.qeph.internal_work[0]));
    twork=std::max(twork,std::abs(next.diagnostics.t3.internal_work[0]));
    ASSERT_TRUE(PublishFeedback(r,p,next)); native.Accept(truth); accepted=next;
    EXPECT_EQ(r.owner.accepted().epoch,step+1); EXPECT_EQ(r.owner.accepted().time,(step+1)*H);
    const std::array<fe::NodalAllocationInfo,4> current{
      r.owner.allocations(),r.qeph.allocations(),r.t3.allocations(),r.publication.allocations()};
    for(unsigned i=0;i<4;++i) {
      EXPECT_EQ(current[i].device_bytes,allocation[i].device_bytes);
      EXPECT_EQ(current[i].device_allocations,allocation[i].device_allocations);
    }
  }
  EXPECT_GT(qforce,1e-5); EXPECT_GT(tforce,1e-5); EXPECT_GT(qwork,1e-12); EXPECT_GT(twork,1e-12);
  Property("maximum_qeph_force_N",qforce); Property("maximum_t3_force_N",tforce);
  Property("maximum_qeph_work_J",qwork); Property("maximum_t3_work_J",twork);
  RecordProperty("native_qeph_intervals",4); RecordProperty("native_t3_intervals",4);
  RecordProperty("aggregate_energy_scale_J","0.000001"); RecordProperty("long_trajectory_admitted","false");
}

TEST_F(MixedShellCuda, CoupledUsageAndBothAttemptCachesAreRequiredBeforeJointPublication) {
  for(unsigned family=0;family<2;++family) {
    SCOPED_TRACE(family);
    Rig r; ASSERT_TRUE(r.PrepareReference());
    ASSERT_EQ(r.initial.Initialize(r.owner).status,fe::NodalStatus::Ok);
    q::QephBatchConfig qc; qc.owner=r.owner.accepted(); qc.element_count=1;
    qc.configuration_id=FeedbackConfiguration; qc.qualification_id=FeedbackQualification;
    qc.usage=family?q::BatchUsage::CoupledForces:q::BatchUsage::PrescribedFields;
    t::T3BatchConfig tc; tc.owner=r.owner.accepted(); tc.element_count=1;
    tc.configuration_id=FeedbackConfiguration; tc.qualification_id=FeedbackQualification;
    tc.usage=family?t::BatchUsage::PrescribedFields:t::BatchUsage::CoupledForces;
    ASSERT_EQ(r.qeph.InitializeJoined(qc,r.binding).status,q::BatchStatus::Success);
    ASSERT_EQ(r.t3.InitializeJoined(tc,r.binding).status,t::BatchStatus::Success);
    fe::NodalTrialToken token; fe::NodalAssemblyView view;
    ASSERT_EQ(r.owner.BeginTrial(&token,&view).status,fe::NodalStatus::Ok);
    ASSERT_EQ(r.qeph.AssembleAccepted(view).status,q::BatchStatus::Success);
    ASSERT_EQ(r.t3.AssembleAccepted(view).status,t::BatchStatus::Success);
    r.Discard();
    EXPECT_EQ(r.publication.Initialize(r.owner,r.qeph,r.t3).status,S::InvalidInput);
    EXPECT_EQ(r.publication.allocations().device_allocations,0u);
  }
  Rig r; ASSERT_TRUE(InitializeFeedback(r)); NativePair native; ASSERT_TRUE(native.Initialize(r));
  ASSERT_TRUE(Prime(r,native));
  Snapshot before; Staged accepted; fe::ShellBatchDiagnostics common;
  ASSERT_TRUE(Read(r.owner,before)); ASSERT_TRUE(Accepted(r,accepted));
  ASSERT_EQ(r.publication.CopyAcceptedDiagnostics(r.owner.accepted(),&common).status,S::Success);
  for(unsigned contributors:{1u,2u}) {
    SCOPED_TRACE(contributors);
    Prepared p; ASSERT_TRUE(PrepareFeedback(r,{},p,contributors));
    auto qd=accepted.diagnostics.qeph; auto td=accepted.diagnostics.t3;
    const auto qsaved=Bytes(qd); const auto tsaved=Bytes(td);
    EXPECT_EQ(r.qeph.EvaluateCandidate(p.view,&qd).status,
              contributors==1?q::BatchStatus::Success:q::BatchStatus::StaleTrial);
    EXPECT_EQ(r.t3.EvaluateCandidate(p.view,&td).status,
              contributors==2?t::BatchStatus::Success:t::BatchStatus::StaleTrial);
    if(contributors==2)EXPECT_EQ(Bytes(qd),qsaved);
    if(contributors==1)EXPECT_EQ(Bytes(td),tsaved);
    auto output=common; const auto held=Bytes(output);
    EXPECT_EQ(r.publication.Prepare(r.owner,p.token,qd,td,&output).status,S::StaleTrial);
    EXPECT_EQ(Bytes(output),held);
    ASSERT_NO_FATAL_FAILURE(PreservedFeedback(r,before,accepted,common));
  }
  for(unsigned family=0;family<2;++family) {
    Prepared p; Staged next; ASSERT_TRUE(PrepareFeedback(r,{},p)); ASSERT_TRUE(Evaluate(r,p,next));
    auto changed=next.diagnostics;
    if(family)changed.t3.accepted_force_assembled=false; else changed.qeph.accepted_force_assembled=false;
    EXPECT_EQ(r.publication.Commit(r.owner,p.token,changed,Receipt(next)).status,S::StaleTrial);
    ASSERT_NO_FATAL_FAILURE(PreservedFeedback(r,before,accepted,common));
  }
  Prepared retry; Staged next; NativeTrials truth;
  ASSERT_TRUE(PrepareFeedback(r,{},retry)); ASSERT_TRUE(Evaluate(r,retry,next,true));
  ASSERT_TRUE(native.Check(r,retry,next,truth)); CheckFeedback(r,before,accepted,retry,next);
  ASSERT_TRUE(PublishFeedback(r,retry,next)); EXPECT_EQ(r.owner.accepted().epoch,2u);
  RecordProperty("native_qeph_intervals",2); RecordProperty("native_t3_intervals",2);
}

TEST_F(MixedShellCuda, CoupledLateFamilyFailurePreservesFeedbackHistoryAndExactRetry) {
  Rig r,clean; ASSERT_TRUE(InitializeFeedback(r)); ASSERT_TRUE(InitializeFeedback(clean));
  NativePair native,other; ASSERT_TRUE(native.Initialize(r)); ASSERT_TRUE(other.Initialize(clean));
  ASSERT_TRUE(Prime(r,native)); ASSERT_TRUE(Prime(clean,other));
  Snapshot before; Staged accepted; fe::ShellBatchDiagnostics common;
  ASSERT_TRUE(Read(r.owner,before)); ASSERT_TRUE(Accepted(r,accepted));
  ASSERT_EQ(r.publication.CopyAcceptedDiagnostics(r.owner.accepted(),&common).status,S::Success);
  ASSERT_GT(std::abs(accepted.diagnostics.qeph.internal_work[0]),1e-12);
  ASSERT_GT(std::abs(accepted.diagnostics.t3.internal_work[0]),1e-12);
  for(unsigned t3_first=0;t3_first<2;++t3_first) {
    SCOPED_TRACE(t3_first);
    Prepared p; ASSERT_TRUE(PrepareFeedback(r,{},p));
    auto qd=accepted.diagnostics.qeph; auto td=accepted.diagnostics.t3;
    const auto& x=p.endpoint.x;
    if(!t3_first) {
      ASSERT_EQ(r.qeph.EvaluateCandidate(p.view,&qd).status,q::BatchStatus::Success);
      const auto held=Bytes(td);
      SetPosition<<<1,1,0,p.view.stream>>>(const_cast<double*>(p.view.kinematics.position_xyz),4,x[3],x[4],x[5]);
      ASSERT_EQ(cudaStreamSynchronize(p.view.stream),cudaSuccess);
      EXPECT_EQ(r.t3.EvaluateCandidate(p.view,&td).status,t::BatchStatus::ElementFailure); EXPECT_EQ(Bytes(td),held);
    } else {
      ASSERT_EQ(r.t3.EvaluateCandidate(p.view,&td).status,t::BatchStatus::Success);
      const auto held=Bytes(qd);
      SetPosition<<<1,1,0,p.view.stream>>>(const_cast<double*>(p.view.kinematics.position_xyz),0,x[3],x[4],x[5]);
      SetPosition<<<1,1,0,p.view.stream>>>(const_cast<double*>(p.view.kinematics.position_xyz),3,x[6],x[7],x[8]);
      ASSERT_EQ(cudaStreamSynchronize(p.view.stream),cudaSuccess);
      EXPECT_EQ(r.qeph.EvaluateCandidate(p.view,&qd).status,q::BatchStatus::ElementFailure); EXPECT_EQ(Bytes(qd),held);
    }
    auto output=common; const auto held=Bytes(output);
    EXPECT_EQ(r.publication.Prepare(r.owner,p.token,qd,td,&output).status,S::StaleTrial);
    EXPECT_EQ(Bytes(output),held);
    ASSERT_NO_FATAL_FAILURE(PreservedFeedback(r,before,accepted,common));
  }
  Prepared retry,reference; Staged next,expected; NativeTrials truth,other_truth;
  ASSERT_TRUE(PrepareFeedback(r,{},retry)); ASSERT_TRUE(Evaluate(r,retry,next,true));
  ASSERT_TRUE(PrepareFeedback(clean,{},reference)); ASSERT_TRUE(Evaluate(clean,reference,expected));
  EXPECT_EQ(retry.endpoint.x,reference.endpoint.x); EXPECT_EQ(retry.endpoint.v,reference.endpoint.v);
  EXPECT_EQ(retry.endpoint.q,reference.endpoint.q); EXPECT_EQ(retry.endpoint.omega,reference.endpoint.omega);
  ExactResults(next,expected); EXPECT_EQ(Bytes(next.diagnostics.kinetic),Bytes(expected.diagnostics.kinetic));
  ASSERT_TRUE(native.Check(r,retry,next,truth)); ASSERT_TRUE(other.Check(clean,reference,expected,other_truth));
  CheckFeedback(r,before,accepted,retry,next);
  ASSERT_TRUE(PublishFeedback(r,retry,next)); ASSERT_TRUE(PublishFeedback(clean,reference,expected));
  EXPECT_EQ(r.owner.accepted().epoch,2u); EXPECT_EQ(clean.owner.accepted().epoch,2u);
  RecordProperty("native_qeph_intervals",4); RecordProperty("native_t3_intervals",4);
}
} // namespace mixed_feedback_test
