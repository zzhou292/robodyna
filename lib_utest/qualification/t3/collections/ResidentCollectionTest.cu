#include "ResidentCollectionFixture.h"
#include "../../active_shell_collection/StorageExpectations.h"
#include "lib_src/elements/qeph/QephBatchStorage.h"
#include "lib_src/elements/t3/T3BatchStorage.h"
#include "lib_src/elements/ShellBatchPublicationStorage.h"

namespace resident_collection_test {
namespace {
using S=fe::ShellPublicationStatus;
std::array<fe::NodalAllocationInfo,4> Allocations(const Rig& r) {
  return {r.owner.allocations(),r.qeph.allocations(),r.t3.allocations(),r.publication.allocations()};
}
void SameAllocations(const Rig& r,const std::array<fe::NodalAllocationInfo,4>& before) {
  const auto after=Allocations(r);
  for(unsigned i=0;i<4;++i) {
    EXPECT_EQ(after[i].device_bytes,before[i].device_bytes);
    EXPECT_EQ(after[i].device_allocations,before[i].device_allocations);
  }
}
void ReportAllocations(const Rig& r) {
  const auto a=Allocations(r);
  EXPECT_EQ(a[0].device_allocations,6u);
  for(unsigned i=1;i<4;++i) EXPECT_EQ(a[i].device_allocations,1u);
  EXPECT_EQ(a[1].device_bytes,active_shell_test::QBytes(QCount,Nodes));
  EXPECT_EQ(a[2].device_bytes,active_shell_test::TBytes(TCount,Nodes));
  EXPECT_EQ(a[3].device_bytes,active_shell_test::PublicationBytes(Nodes));
  EXPECT_LE(a[0].device_bytes,fe::MaxTranslationDeviceBytes);
  EXPECT_LE(a[1].device_bytes,q::MaxBatchDeviceBytes); EXPECT_LE(a[2].device_bytes,t::MaxBatchDeviceBytes);
  Property("owner_device_bytes",a[0].device_bytes); Property("qeph_device_bytes",a[1].device_bytes);
  Property("t3_device_bytes",a[2].device_bytes); Property("common_kinetic_device_bytes",a[3].device_bytes);
  Property("test_pointer_load_device_bytes",6*Capacity*sizeof(double));
  Property("production_device_allocations",9); Property("nodes",Nodes);
  Property("q4_parents",QCount); Property("t3_parents",TCount);
  ::testing::Test::RecordProperty("scope","synthetic collection capacity and short feedback; not Yaris geometry or long dynamics");
}
void Identity(const Rig& r,const Prepared& p,const Staged& s) {
  EXPECT_TRUE(s.diagnostics.valid);
  auto check=[&](const auto& d) {
    EXPECT_TRUE(d.valid); EXPECT_TRUE(d.has_completed_interval); EXPECT_TRUE(d.accepted_force_assembled);
    EXPECT_FALSE(d.kinetic_available); EXPECT_EQ(d.kinetic_translation,0); EXPECT_EQ(d.kinetic_rotation,0);
    EXPECT_EQ(d.kinetic_physical_isotropic,0); EXPECT_EQ(d.kinetic_added_isotropic,0);
    EXPECT_EQ(d.owner_id,r.owner.accepted().owner_id); EXPECT_EQ(d.configuration_id,Configuration);
    EXPECT_EQ(d.qualification_id,Qualification); EXPECT_EQ(d.base_epoch,r.owner.accepted().epoch);
    EXPECT_EQ(d.epoch,d.base_epoch+1); EXPECT_EQ(d.attempt,p.view.attempt);
    EXPECT_EQ(d.time,p.view.proposed_time); EXPECT_EQ(d.velocity_time,p.view.velocity_time);
    EXPECT_EQ(d.kick_dt,d.base_epoch?H:H/2);
  };
  check(s.diagnostics.qeph); check(s.diagnostics.t3);
}
bool Prime(Rig& r,NativeSequence& native) {
  Snapshot base; auto accepted=std::make_unique<Staged>(),next=std::make_unique<Staged>(); Prepared p;
  if(!Read(r.owner,base)||!Accepted(r,*accepted)||!Prepare(r,Pulse(r),*accepted,p)||!Evaluate(r,p,*next)||
     !native.Check(r,p,*next)) return false;
  Identity(r,p,*next); CheckLedgers(r,base,*accepted,p,*next);
  if(!Publish(r,p,*next)) return false;
  native.Accept(); return true;
}
__global__ void SetPosition(double* x,std::size_t node,tl::math::Vec3 value) {
  x[3*node]=value.x; x[3*node+1]=value.y; x[3*node+2]=value.z;
}
void Preserved(Rig& r,const Snapshot& state,const Staged& saved) {
  Snapshot now; ASSERT_TRUE(Read(r.owner,now)); SameState(state,now);
  auto cache=std::make_unique<Staged>(); ASSERT_TRUE(Accepted(r,*cache)); ExactResults(saved,*cache);
  EXPECT_EQ(Bytes(saved.diagnostics),Bytes(cache->diagnostics));
}
}

TEST_F(CudaTest, CollectionCountsHighNodeMassAndReadbackCapacityRejectBeforePublication) {
  auto r=std::make_unique<Rig>(); ASSERT_TRUE(r->Initialize());
  ASSERT_NO_FATAL_FAILURE(CheckReference(*r));
  ASSERT_NO_FATAL_FAILURE(ReportAllocations(*r));
  for(unsigned kind=0;kind<2;++kind) {
    SCOPED_TRACE(kind); q::QephBatch qb; t::T3Batch tb;
    auto qc=r->QConfig(); auto tc=r->TConfig();
    qc.element_count=kind?QCount+1:QCount-1; tc.element_count=kind?TCount+1:TCount-1;
    EXPECT_EQ(qb.InitializeJoined(qc,r->binding).status,q::BatchStatus::InvalidInput);
    EXPECT_EQ(tb.InitializeJoined(tc,r->binding).status,t::BatchStatus::InvalidInput);
    EXPECT_EQ(qb.allocations().device_allocations,0u); EXPECT_EQ(tb.allocations().device_allocations,0u);
    ASSERT_EQ(qb.InitializeJoined(r->QConfig(),r->binding).status,q::BatchStatus::Success);
    ASSERT_EQ(tb.InitializeJoined(r->TConfig(),r->binding).status,t::BatchStatus::Success);
  }
  Snapshot sentinel; sentinel.x.fill(17); sentinel.stamp.epoch=91; const auto held=Bytes(sentinel);
  auto short_buffer=sentinel.buffer(); short_buffer.capacity_nodes=Nodes-1;
  EXPECT_EQ(r->owner.CopyAccepted(short_buffer,&sentinel.stamp).status,fe::NodalStatus::ResourceLimit);
  EXPECT_EQ(Bytes(sentinel),held); ASSERT_TRUE(Read(r->owner,sentinel)); EXPECT_EQ(sentinel.stamp.node_count,Nodes);
  auto bad=std::make_unique<Rig>(); ASSERT_TRUE(bad->BuildReference());
  bad->inverse[116]*=2; ASSERT_TRUE(bad->InitializeOwner()); ASSERT_TRUE(bad->InitializeParticipants());
  // Node 116 is absent from every Q4. Both families must validate the complete
  // collection mass, independently on fresh attempts with clean sticky state.
  for(unsigned family=0;family<2;++family) {
    fe::NodalTrialToken token; fe::NodalAssemblyView view;
    ASSERT_EQ(bad->owner.BeginTrial(&token,&view).status,fe::NodalStatus::Ok);
    if(!family) {
      const auto report=bad->qeph.AssembleAccepted(view);
      EXPECT_EQ(report.status,q::BatchStatus::InvalidMass); EXPECT_EQ(report.node,116u);
    } else {
      const auto report=bad->t3.AssembleAccepted(view);
      EXPECT_EQ(report.status,t::BatchStatus::InvalidMass); EXPECT_EQ(report.node,116u);
    }
    EXPECT_EQ(bad->owner.SealAssembly(token).status,fe::NodalStatus::ContributorFailure); bad->Discard();
  }
  EXPECT_EQ(bad->owner.accepted().epoch,0u);
  EXPECT_NE(bad->publication.Initialize(bad->owner,bad->qeph,bad->t3).status,S::Success);
  EXPECT_EQ(bad->publication.allocations().device_allocations,0u);
}

TEST_F(CudaTest, HundredSeventeenNodesConsumeAllNativeCachesAndGlobalKineticOnce) {
  auto r=std::make_unique<Rig>(); auto native=std::make_unique<NativeSequence>();
  ASSERT_TRUE(r->Initialize()); ASSERT_TRUE(native->Initialize(*r));
  ASSERT_NO_FATAL_FAILURE(CheckReference(*r));
  ASSERT_NO_FATAL_FAILURE(ReportAllocations(*r));
  const auto allocations=Allocations(*r);
  auto cache=std::make_unique<Staged>(),next=std::make_unique<Staged>(); ASSERT_TRUE(Accepted(*r,*cache));
  for(unsigned step=0;step<4;++step) {
    SCOPED_TRACE(step); Snapshot base; Prepared p;
    ASSERT_TRUE(Read(r->owner,base)); ASSERT_TRUE(Prepare(*r,step?Loads{}:Pulse(*r),*cache,p));
    ASSERT_TRUE(Evaluate(*r,p,*next,step%2)); ASSERT_TRUE(native->Check(*r,p,*next));
    Identity(*r,p,*next); CheckLedgers(*r,base,*cache,p,*next);
    if(step==1) {
      // Each boundary node excludes the other family. The complete accepted
      // sum is reconstructed independently; no selected-cell proxy can hide
      // a missing tail contributor at global node 116.
      long double force[2][3]{};
      for(unsigned e=0;e<QCount;++e) for(unsigned local=0;local<4;++local)
        if(r->binding.qeph_nodes(e)[local]==0) {
          const auto f=cache->qeph[e].internal_force[local]; force[0][0]+=f.x; force[0][1]+=f.y; force[0][2]+=f.z;
        }
      for(unsigned e=0;e<TCount;++e) for(unsigned local=0;local<3;++local)
        if(r->binding.t3_nodes(e)[local]==116) {
          const auto f=cache->t3[e].internal_force[local]; force[1][0]+=f.x; force[1][1]+=f.y; force[1][2]+=f.z;
        }
      for(unsigned family=0;family<2;++family) {
        const auto node=family?116:0; long double ratio=0;
        for(unsigned a=0;a<3;++a) {
          const auto delta=std::abs(H*force[family][a]/r->binding.nodes()[node].native.mass);
          const auto budget=temporal::ArithmeticTolerance*(1+std::abs(p.endpoint.v[3*node+a]));
          ratio=std::max(ratio,delta/budget);
        }
        EXPECT_GT(ratio,32); Property(family?"omitted_t3_velocity_budget_ratio":"omitted_q4_velocity_budget_ratio",double(ratio));
      }
      EXPECT_NE(next->diagnostics.qeph.internal_kick_work,0); EXPECT_NE(next->diagnostics.t3.internal_kick_work,0);
    }
    ASSERT_TRUE(Publish(*r,p,*next)); native->Accept();
    ASSERT_TRUE(Accepted(*r,*cache)); ExactResults(*next,*cache);
    EXPECT_EQ(r->owner.accepted().epoch,step+1); EXPECT_EQ(r->owner.accepted().time,(step+1)*H);
    SameAllocations(*r,allocations);
  }
  EXPECT_EQ(cache->qeph.back().proposed_history.stamp().sample_index,4u);
  EXPECT_EQ(cache->t3.back().proposed_history.stamp().sample_index,4u);
  RecordProperty("native_qeph_intervals",4*QCount); RecordProperty("native_t3_intervals",4*TCount);
  RecordProperty("long_trajectory_admitted","false");
}

TEST_F(CudaTest, FinalParentFailureInEitherFamilyPreservesEveryHistoryAndExactRetry) {
  auto r=std::make_unique<Rig>(),clean=std::make_unique<Rig>();
  auto native=std::make_unique<NativeSequence>(),other=std::make_unique<NativeSequence>();
  ASSERT_TRUE(r->Initialize()); ASSERT_TRUE(clean->Initialize());
  ASSERT_TRUE(native->Initialize(*r)); ASSERT_TRUE(other->Initialize(*clean));
  ASSERT_TRUE(Prime(*r,*native)); ASSERT_TRUE(Prime(*clean,*other));
  Snapshot before; ASSERT_TRUE(Read(r->owner,before));
  auto cache=std::make_unique<Staged>(); ASSERT_TRUE(Accepted(*r,*cache));
  EXPECT_NE(cache->qeph.back().proposed_history.data().internal_work[0],0);
  EXPECT_NE(cache->t3.back().proposed_history.data().internal_work[0],0);
  const auto allocations=Allocations(*r);
  for(unsigned family=0;family<2;++family) {
    SCOPED_TRACE(family); Prepared p; ASSERT_TRUE(Prepare(*r,{},*cache,p));
    auto qd=cache->diagnostics.qeph; auto td=cache->diagnostics.t3;
    if(!family) {
      ASSERT_EQ(r->qeph.EvaluateCandidate(p.view,&qd).status,q::BatchStatus::Success);
      const auto held=Bytes(td); const auto& nodes=r->binding.t3_nodes(TCount-1);
      // The first half of this final square remains a valid triangle. Only
      // the last T3 has two coincident corners, after 15 preceding evaluations.
      const auto source=nodes[2],target=nodes[1];
      SetPosition<<<1,1,0,p.view.stream>>>(const_cast<double*>(p.view.kinematics.position_xyz),target,
        {p.endpoint.x[3*source],p.endpoint.x[3*source+1],p.endpoint.x[3*source+2]});
      ASSERT_EQ(cudaStreamSynchronize(p.view.stream),cudaSuccess);
      const auto report=r->t3.EvaluateCandidate(p.view,&td);
      EXPECT_EQ(report.status,t::BatchStatus::ElementFailure); EXPECT_EQ(report.element,TCount-1);
      EXPECT_EQ(Bytes(td),held);
    } else {
      ASSERT_EQ(r->t3.EvaluateCandidate(p.view,&td).status,t::BatchStatus::Success);
      const auto held=Bytes(qd); const auto& nodes=r->binding.qeph_nodes(QCount-1);
      // This top-row corner appears only in the final Q4 of its family.
      const auto source=nodes[1],target=nodes[2];
      SetPosition<<<1,1,0,p.view.stream>>>(const_cast<double*>(p.view.kinematics.position_xyz),target,
        {p.endpoint.x[3*source],p.endpoint.x[3*source+1],p.endpoint.x[3*source+2]});
      ASSERT_EQ(cudaStreamSynchronize(p.view.stream),cudaSuccess);
      const auto report=r->qeph.EvaluateCandidate(p.view,&qd);
      EXPECT_EQ(report.status,q::BatchStatus::ElementFailure); EXPECT_EQ(report.element,QCount-1);
      EXPECT_EQ(Bytes(qd),held);
    }
    auto common=cache->diagnostics; const auto held=Bytes(common);
    EXPECT_EQ(r->publication.Prepare(r->owner,p.token,qd,td,&common).status,S::StaleTrial);
    EXPECT_EQ(Bytes(common),held); ASSERT_NO_FATAL_FAILURE(Preserved(*r,before,*cache));
  }
  Prepared retry,reference; auto next=std::make_unique<Staged>(),expected=std::make_unique<Staged>();
  auto clean_cache=std::make_unique<Staged>(); ASSERT_TRUE(Accepted(*clean,*clean_cache));
  ASSERT_TRUE(Prepare(*r,{},*cache,retry)); ASSERT_TRUE(Evaluate(*r,retry,*next,true));
  ASSERT_TRUE(Prepare(*clean,{},*clean_cache,reference)); ASSERT_TRUE(Evaluate(*clean,reference,*expected));
  SameState(retry.endpoint,reference.endpoint); ExactResults(*next,*expected);
  ASSERT_TRUE(native->Check(*r,retry,*next)); ASSERT_TRUE(other->Check(*clean,reference,*expected));
  Identity(*r,retry,*next); CheckLedgers(*r,before,*cache,retry,*next);
  ASSERT_TRUE(Publish(*r,retry,*next)); ASSERT_TRUE(Publish(*clean,reference,*expected));
  Snapshot actual,truth; ASSERT_TRUE(Read(r->owner,actual)); ASSERT_TRUE(Read(clean->owner,truth));
  SameState(actual,truth,false); EXPECT_EQ(actual.stamp.epoch,2u); SameAllocations(*r,allocations);
  RecordProperty("native_qeph_intervals",4*QCount); RecordProperty("native_t3_intervals",4*TCount);
}

TEST_F(CudaTest, SimultaneousFirstAndLastParentFailuresSelectFirstAndRetryDeterministically) {
  auto r=std::make_unique<Rig>(),clean=std::make_unique<Rig>();
  auto native=std::make_unique<NativeSequence>(),other=std::make_unique<NativeSequence>();
  ASSERT_TRUE(r->Initialize()); ASSERT_TRUE(clean->Initialize());
  ASSERT_TRUE(native->Initialize(*r)); ASSERT_TRUE(other->Initialize(*clean));
  ASSERT_TRUE(Prime(*r,*native)); ASSERT_TRUE(Prime(*clean,*other));
  Snapshot before; ASSERT_TRUE(Read(r->owner,before));
  auto cache=std::make_unique<Staged>(),unavailable=std::make_unique<Staged>();
  ASSERT_TRUE(Accepted(*r,*cache));
  const auto allocations=Allocations(*r);
  for(unsigned family=0;family<2;++family) for(unsigned repetition=0;repetition<4;++repetition) {
    SCOPED_TRACE(family);
    SCOPED_TRACE(repetition);
    Prepared p; ASSERT_TRUE(Prepare(*r,{},*cache,p));
    auto qd=cache->diagnostics.qeph; auto td=cache->diagnostics.t3;
    auto move=[&](std::size_t target,std::size_t source) {
      SetPosition<<<1,1,0,p.view.stream>>>(const_cast<double*>(p.view.kinematics.position_xyz),target,
        {p.endpoint.x[3*source],p.endpoint.x[3*source+1],p.endpoint.x[3*source+2]});
    };
    *unavailable=*cache;
    if(!family) {
      ASSERT_EQ(r->t3.EvaluateCandidate(p.view,&td).status,t::BatchStatus::Success);
      const auto& low=r->binding.qeph_nodes(0); const auto& high=r->binding.qeph_nodes(QCount-1);
      move(low[0],low[1]); move(high[2],high[1]);
      ASSERT_EQ(cudaStreamSynchronize(p.view.stream),cudaSuccess);
      auto first=QInterval(*r,0,p),last=QInterval(*r,QCount-1,p);
      first.position_endpoint[0]=first.position_endpoint[1];
      last.position_endpoint[2]=last.position_endpoint[1];
      q::ForceTrial ignored;
      const auto expected=q::EvaluateForce(r->binding.qeph_reference(0),cache->qeph[0].proposed_history,first,ignored);
      ASSERT_NE(expected,q::Status::kSuccess);
      ASSERT_NE(q::EvaluateForce(r->binding.qeph_reference(QCount-1),cache->qeph.back().proposed_history,last,ignored),q::Status::kSuccess);
      const auto held=Bytes(qd);
      const auto report=r->qeph.EvaluateCandidate(p.view,&qd);
      EXPECT_EQ(report.status,q::BatchStatus::ElementFailure); EXPECT_EQ(report.element,0u);
      EXPECT_EQ(report.element_status,expected); EXPECT_EQ(Bytes(qd),held);
      EXPECT_NE(r->qeph.CopyPreparedResults(qd,unavailable->qeph.data(),QCount).status,q::BatchStatus::Success);
    } else {
      ASSERT_EQ(r->qeph.EvaluateCandidate(p.view,&qd).status,q::BatchStatus::Success);
      const auto& low=r->binding.t3_nodes(0); const auto& high=r->binding.t3_nodes(TCount-1);
      move(low[1],low[0]); move(high[1],high[2]);
      ASSERT_EQ(cudaStreamSynchronize(p.view.stream),cudaSuccess);
      auto first=TInterval(*r,0,p),last=TInterval(*r,TCount-1,p);
      first.position[1]=first.position[0]; last.position[1]=last.position[2];
      t::ForceTrial ignored;
      const auto expected=t::EvaluateForce(r->binding.t3_reference(0),cache->t3[0].proposed_history,first,ignored);
      ASSERT_NE(expected,t::Status::kSuccess);
      ASSERT_NE(t::EvaluateForce(r->binding.t3_reference(TCount-1),cache->t3.back().proposed_history,last,ignored),t::Status::kSuccess);
      const auto held=Bytes(td);
      const auto report=r->t3.EvaluateCandidate(p.view,&td);
      EXPECT_EQ(report.status,t::BatchStatus::ElementFailure); EXPECT_EQ(report.element,0u);
      EXPECT_EQ(report.element_status,expected); EXPECT_EQ(Bytes(td),held);
      EXPECT_NE(r->t3.CopyPreparedResults(td,unavailable->t3.data(),TCount).status,t::BatchStatus::Success);
    }
    ExactResults(*cache,*unavailable);
    auto common=cache->diagnostics; const auto held=Bytes(common);
    EXPECT_EQ(r->publication.Prepare(r->owner,p.token,qd,td,&common).status,S::StaleTrial);
    EXPECT_EQ(Bytes(common),held);
    ASSERT_NO_FATAL_FAILURE(Preserved(*r,before,*cache));
    SameAllocations(*r,allocations);
  }
  Prepared retry,reference; auto next=std::make_unique<Staged>(),expected=std::make_unique<Staged>();
  auto clean_cache=std::make_unique<Staged>(); ASSERT_TRUE(Accepted(*clean,*clean_cache));
  ASSERT_TRUE(Prepare(*r,{},*cache,retry)); ASSERT_TRUE(Evaluate(*r,retry,*next,true));
  ASSERT_TRUE(Prepare(*clean,{},*clean_cache,reference)); ASSERT_TRUE(Evaluate(*clean,reference,*expected));
  SameState(retry.endpoint,reference.endpoint); ExactResults(*next,*expected);
  ASSERT_TRUE(native->Check(*r,retry,*next)); ASSERT_TRUE(other->Check(*clean,reference,*expected));
  ASSERT_TRUE(Publish(*r,retry,*next)); ASSERT_TRUE(Publish(*clean,reference,*expected));
  Snapshot actual,truth; ASSERT_TRUE(Read(r->owner,actual)); ASSERT_TRUE(Read(clean->owner,truth));
  SameState(actual,truth,false); SameAllocations(*r,allocations);
  RecordProperty("simultaneous_failure_attempts",8);
  RecordProperty("native_qeph_intervals",4*QCount); RecordProperty("native_t3_intervals",4*TCount);
}
} // namespace resident_collection_test
