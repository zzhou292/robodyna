#include "T3BatchLedger.h"

namespace t3_batch_test {
static __global__ void Set(double* values,unsigned i,double value) { values[i]=value; }
TEST_F(T3BatchCuda, InvalidReferenceMassAndActualRestNeverAuthorizeStartupCache) {
  Rig r; ASSERT_TRUE(r.InitializeOwner());
  for(unsigned kind=0;kind<11;++kind) {
    SCOPED_TRACE(kind); t::T3Batch batch; auto config=r.Config(); auto elements=r.element;
    if(kind==0) config.usage=t::BatchUsage::Unspecified;
    if(kind==1) config.element_count=3;
    if(kind==2) config.max_device_bytes=1;
    if(kind==3) config.owner.node_count=17;
    if(kind==4) elements[1].reference.area*=2;
    if(kind==5) {
      auto in=elements[1].reference.input;
      // Keep this triangle's source IDs distinct while breaking its shared
      // global node1 identity. Incrementing by one aliases its native node2.
      in.node_ids[0]+=1000;
      ASSERT_EQ(t::InitializeReference(in,elements[1].reference),t::Status::kSuccess);
    }
    if(kind==6) elements[1].nodes[2]=elements[1].nodes[0];
    if(kind==7) elements[1]=elements[0];
    if(kind==8) config.owner.node_count=5; // Uncovered node.
    if(kind==9) elements[1].reference.element_added_inertia=std::nextafter(elements[1].reference.element_added_inertia,1.);
    if(kind==10) elements[1].reference.startup_derivative[0]=1;
    const auto rejected=batch.Initialize(config,elements.data());
    EXPECT_NE(rejected.status,t::BatchStatus::Success);
    if(kind==5) {
      EXPECT_EQ(rejected.status,t::BatchStatus::InvalidInput);
      EXPECT_EQ(rejected.element,1u); EXPECT_EQ(rejected.node,1u);
    }
    EXPECT_EQ(batch.allocations().device_allocations,0u);
    ASSERT_EQ(batch.Initialize(r.Config(),r.element.data()).status,t::BatchStatus::Success);
  }
  for(unsigned kind=0;kind<5;++kind) {
    SCOPED_TRACE(kind); Rig bad(1);
    if(kind==0) bad.inverse[2]*=2;
    if(kind==1) bad.inverse_j[2]*=2;
    if(kind==2) bad.x[0]+=.125;
    if(kind==3) bad.zero[4]=.125;
    if(kind==4) { bad.fixed[2]=7; bad.rotation_fixed[2]=1; bad.inverse[2]=0; bad.inverse_j[2]=0; }
    ASSERT_TRUE(bad.Initialize()); Snapshot before; ASSERT_TRUE(Read(bad.owner,before));
    fe::NodalTrialToken token; fe::NodalAssemblyView view;
    ASSERT_EQ(bad.owner.BeginTrial(&token,&view).status,fe::NodalStatus::Ok);
    EXPECT_EQ(bad.batch.AssembleAccepted(view).status,(kind<2||kind==4)?t::BatchStatus::InvalidMass:t::BatchStatus::InvalidInput);
    EXPECT_EQ(bad.owner.SealAssembly(token).status,fe::NodalStatus::ContributorFailure);
    bad.owner.Discard(); bad.batch.DiscardTrial(); Snapshot after; ASSERT_TRUE(Read(bad.owner,after)); SameState(before,after);
    Results out{}; t::BatchDiagnostics d; const auto bytes=Bytes(out); const auto diagnostic=Bytes(d);
    EXPECT_EQ(bad.batch.CopyAcceptedResults(bad.owner.accepted(),out.data(),out.size(),&d).status,t::BatchStatus::NotBound);
    EXPECT_EQ(Bytes(out),bytes); EXPECT_EQ(Bytes(d),diagnostic);
  }
}

TEST_F(T3BatchCuda, ForeignStreamPhaseAndTamperedReceiptPreserveAcceptedAndReadbackBytes) {
  Rig r,foreign; ASSERT_TRUE(r.Initialize()); ASSERT_TRUE(r.Bind());
  ASSERT_TRUE(foreign.Initialize()); ASSERT_TRUE(foreign.Bind());
  Results accepted{}; t::BatchDiagnostics initial; ASSERT_TRUE(Accepted(r,accepted,initial));
  const auto held=Bytes(accepted); const auto initial_bytes=Bytes(initial); Snapshot before; ASSERT_TRUE(Read(r.owner,before));
  auto wrong_stamp=r.owner.accepted(); ++wrong_stamp.epoch;
  EXPECT_EQ(r.batch.CopyAcceptedResults(wrong_stamp,accepted.data(),accepted.size(),&initial).status,t::BatchStatus::StaleTrial);
  EXPECT_EQ(r.batch.CopyAcceptedResults(r.owner.accepted(),accepted.data(),1,&initial).status,t::BatchStatus::ResourceLimit);
  EXPECT_EQ(r.batch.CopyAcceptedResults(r.owner.accepted(),accepted.data(),accepted.size(),
      reinterpret_cast<t::BatchDiagnostics*>(accepted.data())).status,t::BatchStatus::InvalidInput);
  EXPECT_EQ(Bytes(accepted),held); EXPECT_EQ(Bytes(initial),initial_bytes);
  fe::NodalTrialToken token,other_token; fe::NodalPreparedView p,other;
  ASSERT_TRUE(Prepare(r,Schedule(r,0),token,p)); ASSERT_TRUE(Prepare(foreign,Schedule(foreign,0),other_token,other));
  t::BatchDiagnostics d; const auto untouched=Bytes(d);
  EXPECT_EQ(r.batch.EvaluateCandidate(other,&d).status,t::BatchStatus::WrongOwner); EXPECT_EQ(Bytes(d),untouched);
  auto bad=p; bad.base_velocity_time+=1;
  EXPECT_EQ(r.batch.EvaluateCandidate(bad,&d).status,t::BatchStatus::StaleTrial); EXPECT_EQ(Bytes(d),untouched);
  ASSERT_NE(p.stream,other.stream); bad=p; bad.stream=other.stream;
  EXPECT_EQ(r.batch.EvaluateCandidate(bad,&d).status,t::BatchStatus::StaleTrial); EXPECT_EQ(Bytes(d),untouched);
  Results out{}; ASSERT_TRUE(Candidate(r,p,d,out)); const auto output=Bytes(out); const auto diagnostic=Bytes(d);
  for(unsigned kind=0;kind<4;++kind) {
    auto stale=d;
    if(kind==0) ++stale.attempt;
    if(kind==1) ++stale.configuration_id;
    if(kind==2) stale.internal_work[0]=std::nextafter(stale.internal_work[0],1.);
    if(kind==3) stale.accepted_force_assembled=true;
    EXPECT_EQ(r.batch.CopyPreparedResults(stale,out.data(),out.size()).status,t::BatchStatus::StaleTrial);
    EXPECT_EQ(Bytes(out),output);
  }
  EXPECT_EQ(r.batch.CopyPreparedResults(d,out.data(),1).status,t::BatchStatus::ResourceLimit);
  EXPECT_EQ(r.batch.CopyPreparedResults(d,reinterpret_cast<t::ForceTrial*>(&d),out.size()).status,t::BatchStatus::InvalidInput);
  EXPECT_EQ(Bytes(d),diagnostic); EXPECT_EQ(Bytes(out),output);
  auto receipt=Receipt(d); ++receipt.qualification_id;
  EXPECT_EQ(t::CommitT3Trial(r.owner,token,r.batch,d,receipt).status,t::BatchStatus::StaleTrial);
  ASSERT_TRUE(Accepted(r,accepted,initial)); EXPECT_EQ(Bytes(accepted),held); EXPECT_EQ(Bytes(initial),initial_bytes);
  Snapshot after; ASSERT_TRUE(Read(r.owner,after)); SameState(before,after);
  EXPECT_EQ(r.batch.CopyPreparedResults(d,out.data(),out.size()).status,t::BatchStatus::StaleTrial);
  // Matching numeric stamps with foreign physical buffers cannot pass the
  // commit coordinator's comparison to the opaque token's authentic views.
  ASSERT_TRUE(Prepare(r,Schedule(r,0),token,p)); auto fake=p;
  fake.kinematics=other.kinematics; fake.base_kinematics=other.base_kinematics;
  ASSERT_TRUE(Candidate(r,fake,d,out));
  EXPECT_EQ(t::CommitT3Trial(r.owner,token,r.batch,d,Receipt(d)).status,t::BatchStatus::StaleTrial);
  ASSERT_TRUE(Accepted(r,accepted,initial)); EXPECT_EQ(Bytes(accepted),held);
  foreign.owner.Discard(); foreign.batch.DiscardTrial();
  ASSERT_TRUE(Prepare(r,Schedule(r,0),token,p)); ASSERT_TRUE(Candidate(r,p,d,out));
  CheckLedger(r,accepted,out,Endpoint(r,p),d); ASSERT_FALSE(::testing::Test::HasFailure()); ASSERT_TRUE(Commit(r,token,d));
}

TEST_F(T3BatchCuda, LateSecondTriangleFailureLeavesNonzeroAcceptedHistoryAndRetryExactlyIntact) {
  Rig r,clean; ASSERT_TRUE(r.Initialize()); ASSERT_TRUE(clean.Initialize());
  ASSERT_TRUE(r.Bind()); ASSERT_TRUE(clean.Bind());
  Results out{},truth{}; t::BatchDiagnostics d,td; fe::NodalTrialToken token,ct; fe::NodalPreparedView p,cp;
  for(auto* current:{&r,&clean}) {
    Results base; t::BatchDiagnostics initial; ASSERT_TRUE(Accepted(*current,base,initial));
    ASSERT_TRUE(Prepare(*current,Schedule(*current,0),token,p)); ASSERT_TRUE(Candidate(*current,p,d,out));
    CheckLedger(*current,base,out,Endpoint(*current,p),d);
    ASSERT_FALSE(::testing::Test::HasFailure()); ASSERT_TRUE(Commit(*current,token,d));
  }
  Snapshot before; ASSERT_TRUE(Read(r.owner,before)); ASSERT_TRUE(Accepted(r,out,d));
  EXPECT_GT(std::abs(d.internal_work[0]),1e-12); const auto retained=Bytes(out); const auto saved=Bytes(d);
  for(unsigned fault=0;fault<2;++fault) {
    SCOPED_TRACE(fault); ASSERT_TRUE(Prepare(r,Schedule(r,1),token,p)); const auto endpoint=Endpoint(r,p);
    if(!fault) {
      // Only node3 belongs to the second triangle: collapse it onto shared
      // node1, after a real owner advance. The first trial element stays valid.
      for(unsigned a=0;a<3;++a) Set<<<1,1,0,p.stream>>>(const_cast<double*>(p.kinematics.position_xyz),9+a,endpoint.x[3+a]);
    } else Set<<<1,1,0,p.stream>>>(const_cast<double*>(p.kinematics.velocity_xyz),9,std::numeric_limits<double>::max());
    ASSERT_EQ(cudaStreamSynchronize(p.stream),cudaSuccess);
    const auto rejected=r.batch.EvaluateCandidate(p,&d);
    EXPECT_EQ(rejected.status,t::BatchStatus::ElementFailure); EXPECT_EQ(rejected.element,1u); EXPECT_EQ(Bytes(d),saved);
    r.owner.Discard(); r.batch.DiscardTrial(); ASSERT_TRUE(Accepted(r,out,d));
    EXPECT_EQ(Bytes(out),retained); EXPECT_EQ(Bytes(d),saved); Snapshot after; ASSERT_TRUE(Read(r.owner,after)); SameState(before,after);
  }
  const auto accepted=out;
  ASSERT_TRUE(Prepare(r,Schedule(r,1),token,p)); ASSERT_TRUE(Candidate(r,p,d,out));
  ASSERT_TRUE(Prepare(clean,Schedule(clean,1),ct,cp)); ASSERT_TRUE(Candidate(clean,cp,td,truth));
  for(unsigned e=0;e<2;++e) oracle::Exact(out[e],truth[e]);
  CheckLedger(r,accepted,out,Endpoint(r,p),d); CheckLedger(clean,accepted,truth,Endpoint(clean,cp),td);
  ASSERT_FALSE(::testing::Test::HasFailure()); ASSERT_TRUE(Commit(r,token,d)); ASSERT_TRUE(Commit(clean,ct,td));
  EXPECT_EQ(r.owner.accepted().epoch,2u); EXPECT_EQ(clean.owner.accepted().epoch,2u);
}

TEST_F(T3BatchCuda, PendingCudaFailureBeforeCommitPreventsBothSlabPublicationsAndPoisonsBatch) {
  Rig r(1); ASSERT_TRUE(r.Initialize()); ASSERT_TRUE(r.Bind());
  fe::NodalTrialToken token; fe::NodalPreparedView p; ASSERT_TRUE(Prepare(r,Schedule(r,0),token,p));
  t::BatchDiagnostics d; Results out{}; ASSERT_TRUE(Candidate(r,p,d,out)); const auto stamp=r.owner.accepted();
  ASSERT_EQ(cudaPeekAtLastError(),cudaSuccess);
  Noop<<<1,0,0,p.stream>>>(); const auto error=cudaPeekAtLastError();
  ASSERT_TRUE(error==cudaErrorInvalidConfiguration||error==cudaErrorInvalidValue);
  const auto report=t::CommitT3Trial(r.owner,token,r.batch,d,Receipt(d));
  EXPECT_EQ(report.status,t::BatchStatus::NodalFailure); EXPECT_EQ(report.nodal_status,fe::NodalStatus::DeviceFailure);
  SameStamp(r.owner.accepted(),stamp); const auto bytes=Bytes(out); t::BatchDiagnostics held;
  EXPECT_EQ(r.batch.CopyAcceptedResults(stamp,out.data(),out.size(),&held).status,t::BatchStatus::DeviceFailure);
  EXPECT_EQ(Bytes(out),bytes);
  Rig fresh(1); ASSERT_TRUE(fresh.InitializeOwner()); fe::NodalAssemblyView view;
  ASSERT_EQ(fresh.owner.BeginTrial(&token,&view).status,fe::NodalStatus::Ok);
  EXPECT_EQ(r.batch.AssembleAccepted(view).status,t::BatchStatus::DeviceFailure);
  EXPECT_EQ(fresh.owner.SealAssembly(token).status,fe::NodalStatus::ContributorFailure); fresh.owner.Discard();
}
} // namespace t3_batch_test
