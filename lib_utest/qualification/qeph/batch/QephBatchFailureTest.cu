#include "QephBatchFixture.h"
#include <limits>

namespace qeph_batch_test {
TEST_F(QephBatchCuda, InvalidStartupAndActualMassRestMismatchNeverAuthorizeInitialCache) {
  Rig rig(2); ASSERT_TRUE(rig.InitializeOwner());
  for(unsigned kind=0;kind<7;++kind) {
    SCOPED_TRACE(kind);
    q::QephBatch batch; auto config=rig.Config(); auto elements=rig.element;
    if(kind==0) config.usage=q::BatchUsage::Unspecified;
    if(kind==1) config.element_count=5;
    if(kind==2) config.max_device_bytes=1;
    if(kind==3) config.owner.node_count=17;
    if(kind==4) elements[1].reference.area*=2;
    if(kind==5) { auto& input=elements[1].reference.input; input.node_ids[0]+=100;
      ASSERT_EQ(q::InitializeReference(input,elements[1].reference),q::Status::kSuccess); }
    if(kind==6) elements[1].nodes[3]=elements[1].nodes[0];
    EXPECT_NE(batch.Initialize(config,elements.data()).status,q::BatchStatus::Success);
    EXPECT_EQ(batch.allocations().device_allocations,0u);
  }
  for(unsigned kind=0;kind<4;++kind) {
    Rig bad(1);
    if(kind==0) bad.inverse[2]*=2;
    if(kind==1) bad.inverse_j[2]*=2;
    if(kind==2) bad.x[0]+=.125;
    if(kind==3) bad.zero[4]=.125;
    ASSERT_TRUE(bad.Initialize()); Snapshot before; ASSERT_TRUE(Read(bad.owner,before));
    fe::NodalTrialToken token; fe::NodalAssemblyView v;
    ASSERT_EQ(bad.owner.BeginTrial(&token,&v).status,fe::NodalStatus::Ok);
    EXPECT_EQ(bad.batch.AssembleAccepted(v).status,kind<2?q::BatchStatus::InvalidMass:q::BatchStatus::InvalidInput);
    EXPECT_EQ(bad.owner.SealAssembly(token).status,fe::NodalStatus::ContributorFailure);
    bad.owner.Discard(); bad.batch.DiscardTrial(); Snapshot after; ASSERT_TRUE(Read(bad.owner,after)); SameState(before,after);
    std::array<q::ForceTrial,4> out{}; q::BatchDiagnostics d; const auto bytes=Bytes(out);
    EXPECT_EQ(bad.batch.CopyAcceptedResults(bad.owner.accepted(),out.data(),4,&d).status,q::BatchStatus::NotBound);
    EXPECT_EQ(Bytes(out),bytes);
  }
}

TEST_F(QephBatchCuda, ForeignStaleAndTamperedPublicationPreservesAllAcceptedAndCallerBytes) {
  Rig rig(2),foreign(2); ASSERT_TRUE(rig.Initialize()); ASSERT_TRUE(rig.Bind());
  ASSERT_TRUE(foreign.Initialize()); ASSERT_TRUE(foreign.Bind());
  std::array<q::ForceTrial,4> accepted{}; q::BatchDiagnostics initial; ASSERT_TRUE(ReadAccepted(rig,accepted,initial));
  const auto accepted_bytes=Bytes(accepted); const auto initial_bytes=Bytes(initial);
  Snapshot before; ASSERT_TRUE(Read(rig.owner,before));
  auto wrong_stamp=rig.owner.accepted(); ++wrong_stamp.epoch;
  EXPECT_EQ(rig.batch.CopyAcceptedResults(wrong_stamp,accepted.data(),4,&initial).status,q::BatchStatus::StaleTrial);
  EXPECT_EQ(rig.batch.CopyAcceptedResults(rig.owner.accepted(),accepted.data(),1,&initial).status,q::BatchStatus::ResourceLimit);
  EXPECT_EQ(rig.batch.CopyAcceptedResults(rig.owner.accepted(),accepted.data(),4,
      reinterpret_cast<q::BatchDiagnostics*>(accepted.data())).status,q::BatchStatus::InvalidInput);
  EXPECT_EQ(Bytes(accepted),accepted_bytes); EXPECT_EQ(Bytes(initial),initial_bytes);
  fe::NodalTrialToken token,other_token; fe::NodalPreparedView p,other;
  ASSERT_TRUE(Prepare(rig,Pattern(rig),token,p)); ASSERT_TRUE(Prepare(foreign,Pattern(foreign),other_token,other));
  q::BatchDiagnostics d; const auto unchanged=Bytes(d);
  EXPECT_EQ(rig.batch.EvaluateCandidate(other,&d).status,q::BatchStatus::WrongOwner); EXPECT_EQ(Bytes(d),unchanged);
  auto bad=p; bad.base_velocity_time+=1;
  EXPECT_EQ(rig.batch.EvaluateCandidate(bad,&d).status,q::BatchStatus::StaleTrial); EXPECT_EQ(Bytes(d),unchanged);
  std::array<q::ForceTrial,4> result{}; ASSERT_TRUE(Candidate(rig,p,d,result));
  const auto result_bytes=Bytes(result); const auto diagnostic_bytes=Bytes(d);
  for(unsigned kind=0;kind<4;++kind) {
    auto stale=d;
    if(kind==0) ++stale.attempt;
    if(kind==1) ++stale.configuration_id;
    if(kind==2) stale.kinetic_translation=std::nextafter(stale.kinetic_translation,1.);
    if(kind==3) stale.accepted_force_assembled=true;
    EXPECT_EQ(rig.batch.CopyPreparedResults(stale,result.data(),4).status,q::BatchStatus::StaleTrial);
    EXPECT_EQ(Bytes(result),result_bytes);
  }
  EXPECT_EQ(rig.batch.CopyPreparedResults(d,result.data(),1).status,q::BatchStatus::ResourceLimit);
  EXPECT_EQ(rig.batch.CopyPreparedResults(d,reinterpret_cast<q::ForceTrial*>(&d),4).status,q::BatchStatus::InvalidInput);
  EXPECT_EQ(Bytes(d),diagnostic_bytes); EXPECT_EQ(Bytes(result),result_bytes);
  ASSERT_EQ(rig.batch.CopyPreparedResults(d,result.data(),4).status,q::BatchStatus::Success);
  auto receipt=Receipt(d); ++receipt.qualification_id;
  EXPECT_EQ(q::CommitQephTrial(rig.owner,token,rig.batch,d,receipt).status,q::BatchStatus::StaleTrial);
  ASSERT_TRUE(ReadAccepted(rig,accepted,initial)); EXPECT_EQ(Bytes(accepted),accepted_bytes);
  Snapshot after; ASSERT_TRUE(Read(rig.owner,after)); SameState(before,after);
  EXPECT_EQ(rig.batch.CopyPreparedResults(d,result.data(),4).status,q::BatchStatus::StaleTrial);
  // Correct-looking foreign pointers cannot publish: the coordinator re-borrows
  // the opaque token and compares its real views with the evaluated operands.
  auto authentic=d;
  ASSERT_TRUE(Prepare(rig,Pattern(rig),token,p)); auto fake=p;
  fake.kinematics=other.kinematics; fake.base_kinematics=other.base_kinematics;
  ASSERT_TRUE(Candidate(rig,fake,authentic,result));
  EXPECT_EQ(q::CommitQephTrial(rig.owner,token,rig.batch,authentic,Receipt(authentic)).status,q::BatchStatus::StaleTrial);
  ASSERT_TRUE(ReadAccepted(rig,accepted,initial)); EXPECT_EQ(Bytes(accepted),accepted_bytes);
  foreign.owner.Discard(); foreign.batch.DiscardTrial();
  ASSERT_TRUE(Prepare(rig,Pattern(rig),token,p)); ASSERT_TRUE(Candidate(rig,p,d,result)); ASSERT_TRUE(Commit(rig,token,d));
  EXPECT_EQ(rig.owner.accepted().epoch,1u);
}

TEST_F(QephBatchCuda, LateFourthElementFailureDiscardsTrialAndRetryMatchesCleanPrescribedSequence) {
  Rig rig(4,false,true),clean(4,false,true); ASSERT_TRUE(rig.Initialize()); ASSERT_TRUE(clean.Initialize());
  ASSERT_TRUE(rig.Bind()); ASSERT_TRUE(clean.Bind());
  std::array<q::ForceTrial,4> result{},truth{}; q::BatchDiagnostics d,td;
  fe::NodalTrialToken token,ct; fe::NodalPreparedView p,cp;
  for(auto* item:{&rig,&clean}) {
    ASSERT_TRUE(Prepare(*item,Pattern(*item,1,false),token,p)); ASSERT_TRUE(Candidate(*item,p,d,result)); ASSERT_TRUE(Commit(*item,token,d));
  }
  Snapshot before; ASSERT_TRUE(Read(rig.owner,before));
  ASSERT_TRUE(ReadAccepted(rig,result,d)); const auto retained=Bytes(result); const auto saved=Bytes(d);
  Loads collapse{};
  for(unsigned i=0;i<4;++i) for(unsigned a=0;a<3;++a) {
    const auto n=rig.element[3].nodes[i],j=3*n+a;
    collapse.force[j]=rig.mass[n]*((-before.x[j]/rig.h-before.v[j])/rig.h);
  }
  ASSERT_TRUE(Prepare(rig,collapse,token,p));
  const auto rejected=rig.batch.EvaluateCandidate(p,&d);
  EXPECT_EQ(rejected.status,q::BatchStatus::ElementFailure); EXPECT_EQ(rejected.element,3u);
  EXPECT_EQ(Bytes(d),saved);
  rig.owner.Discard(); rig.batch.DiscardTrial();
  ASSERT_TRUE(ReadAccepted(rig,result,d)); EXPECT_EQ(Bytes(result),retained); EXPECT_EQ(Bytes(d),saved);
  Snapshot after; ASSERT_TRUE(Read(rig.owner,after)); SameState(before,after);
  ASSERT_TRUE(Prepare(rig,Pattern(rig,-1,false),token,p)); ASSERT_TRUE(Candidate(rig,p,d,result));
  ASSERT_TRUE(Prepare(clean,Pattern(clean,-1,false),ct,cp)); ASSERT_TRUE(Candidate(clean,cp,td,truth));
  for(unsigned e=0;e<4;++e) qeph_force_port_test::ForceAgreement(result[e],truth[e],rig.element[e].reference.input,ReadInterval(rig,e,p),0.);
  ASSERT_TRUE(Commit(rig,token,d)); ASSERT_TRUE(Commit(clean,ct,td));
  EXPECT_EQ(rig.owner.accepted().epoch,2u); EXPECT_EQ(clean.owner.accepted().epoch,2u);
}

TEST_F(QephBatchCuda, SafeLateLaunchErrorPreventsBothPublicationsAndPoisonsBatch) {
  Rig rig(1); ASSERT_TRUE(rig.Initialize()); ASSERT_TRUE(rig.Bind());
  fe::NodalTrialToken token; fe::NodalPreparedView p; ASSERT_TRUE(Prepare(rig,Pattern(rig),token,p));
  q::BatchDiagnostics d; std::array<q::ForceTrial,4> result{}; ASSERT_TRUE(Candidate(rig,p,d,result));
  const auto stamp=rig.owner.accepted();
  ASSERT_EQ(cudaPeekAtLastError(),cudaSuccess);
  Noop<<<1,0,0,p.stream>>>();
  const auto launch_error=cudaPeekAtLastError();
  ASSERT_TRUE(launch_error==cudaErrorInvalidConfiguration||launch_error==cudaErrorInvalidValue);
  const auto r=q::CommitQephTrial(rig.owner,token,rig.batch,d,Receipt(d));
  EXPECT_EQ(r.status,q::BatchStatus::NodalFailure); EXPECT_EQ(r.nodal_status,fe::NodalStatus::DeviceFailure);
  SameStamp(rig.owner.accepted(),stamp);
  const auto bytes=Bytes(result); q::BatchDiagnostics held;
  EXPECT_EQ(rig.batch.CopyAcceptedResults(stamp,result.data(),4,&held).status,q::BatchStatus::DeviceFailure);
  EXPECT_EQ(Bytes(result),bytes);
  // Recoverable invalid launch, no hardware/context-destroying injection. A
  // poisoned participant must still invalidate otherwise fresh assembly views.
  Rig fresh(1); ASSERT_TRUE(fresh.InitializeOwner()); fe::NodalAssemblyView v;
  ASSERT_EQ(fresh.owner.BeginTrial(&token,&v).status,fe::NodalStatus::Ok);
  EXPECT_EQ(rig.batch.AssembleAccepted(v).status,q::BatchStatus::DeviceFailure);
  EXPECT_EQ(fresh.owner.SealAssembly(token).status,fe::NodalStatus::ContributorFailure);
  fresh.owner.Discard();
}
} // namespace qeph_batch_test
