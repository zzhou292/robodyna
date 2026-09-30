#include "ConnectorPublicationFixture.h"

namespace nodal_mass_test::joined_connector {
class ConnectorPublication:public NodalMassCuda {};
namespace {
temporal::Loads Load() {
  temporal::Loads load;load.force[12]=1000.;load.force[13]=-400.;load.force[14]=200.;
  load.couple[12]=.5;load.couple[13]=-.25;return load;
}
void SameMotion(const temporal::Snapshot& a,const temporal::Snapshot& b) {
  EXPECT_EQ(a.x,b.x);EXPECT_EQ(a.v,b.v);EXPECT_EQ(a.q,b.q);EXPECT_EQ(a.omega,b.omega);
  EXPECT_EQ(a.reaction,b.reaction);EXPECT_EQ(a.couple,b.couple);
  EXPECT_EQ(a.stamp.epoch,b.stamp.epoch);EXPECT_EQ(a.stamp.time,b.stamp.time);
}
}
TEST_F(ConnectorPublication,MovingStartupAnd64LoadedIntervalsUseOneCombinedKineticAndAllCaches) {
  Rig rig;ASSERT_TRUE(rig.Start(true));Accepted old;ASSERT_TRUE(rig.Read(old));
  ASSERT_TRUE(old.common.has_connector);EXPECT_GT(old.common.kinetic.connector_translation,0.);
  CheckKinetic(rig,old.nodes,old.common.kinetic);
  const auto owner_bytes=rig.owner.allocations().device_bytes;
  const auto spring_bytes=rig.connector.allocations().device_bytes;
  const auto common_bytes=rig.publication.allocations().device_bytes;
  for(unsigned step=0;step<64;++step) {
    SCOPED_TRACE(step);Candidate p;ASSERT_TRUE(rig.Begin(p,Load()));
    CheckCacheScatter(rig,old,p,Load());ASSERT_TRUE(rig.Evaluate(p));
    EXPECT_TRUE(p.common.has_connector);EXPECT_FALSE(p.common.qeph.kinetic_available);EXPECT_FALSE(p.common.t3.kinetic_available);
    EXPECT_EQ(p.common.connector.epoch,step+1);EXPECT_EQ(p.common.connector.base_epoch,step);
    EXPECT_EQ(p.common.connector.kick_dt,step?rig.initial.h:.5*rig.initial.h);
    ASSERT_LE(rig.initial.h,.5*p.common.connector.minimum_native_dt);
    CheckKinetic(rig,old.nodes,p.common.base_kinetic);
    ASSERT_TRUE(rig.Commit(p));Accepted next;ASSERT_TRUE(rig.Read(next));
    CheckKinetic(rig,next.nodes,next.common.kinetic);
    EXPECT_EQ(next.common.connector.phase,spring::BatchPhase::Accepted);
    EXPECT_TRUE(spring::batch_detail::SameDiagnostics(next.common.connector,
        [&]{auto d=p.cd;d.phase=spring::BatchPhase::Accepted;return d;}()));
    old=next;
  }
  EXPECT_GT(old.common.kinetic.connector_rotation,0.);
  EXPECT_GT(std::abs(old.connectors[1].history.displacement_m.x)+std::abs(old.connectors[1].history.displacement_m.y),0.);
  EXPECT_EQ(rig.owner.allocations().device_bytes,owner_bytes);
  EXPECT_EQ(rig.connector.allocations().device_bytes,spring_bytes);
  EXPECT_EQ(rig.publication.allocations().device_bytes,common_bytes);
}
TEST_F(ConnectorPublication,MissingContributorAndTamperedCompleteReceiptPreserveEveryAcceptedSlab) {
  Rig rig;ASSERT_TRUE(rig.Start());Candidate first;ASSERT_TRUE(rig.Begin(first,Load()));
  ASSERT_TRUE(rig.Evaluate(first));ASSERT_TRUE(rig.Commit(first));
  Accepted before;ASSERT_TRUE(rig.Read(before));
  for(unsigned fault=0;fault<7;++fault) {
    SCOPED_TRACE(fault);Candidate p;ASSERT_TRUE(rig.Begin(p,Load(),fault!=0));
    auto output=before.common;const auto saved=Bytes(output);
    if(fault==0) {
      ASSERT_EQ(rig.qb.EvaluateCandidate(rig.owner,p.token,p.view,&p.qd).status,q::BatchStatus::Success);
      ASSERT_EQ(rig.tb.EvaluateCandidate(rig.owner,p.token,p.view,&p.td).status,t::BatchStatus::Success);
      EXPECT_EQ(rig.connector.EvaluateCandidate(rig.owner,p.token,p.view,&p.cd).status,CS::StaleTrial);
      EXPECT_NE(rig.publication.Prepare(rig.owner,p.token,p.qd,p.td,p.cd,&output).status,PS::Success);
    } else {
      ASSERT_TRUE(rig.Evaluate(p));
      if(fault==1) {
        EXPECT_EQ(rig.publication.Prepare(rig.owner,p.token,p.qd,p.td,&output).status,PS::NotJoined);
      } else {
        auto expected=p.common;
        if(fault==2)expected.has_connector=false;
        if(fault==3)++expected.connector.attempt;
        if(fault==4)expected.kinetic.connector_translation=std::nextafter(expected.kinetic.connector_translation,1.);
        if(fault==5)expected.connector.internal_work_J[3]=std::nextafter(expected.connector.internal_work_J[3],1.);
        if(fault==6)rig.connector.DiscardTrial();
        EXPECT_EQ(rig.publication.Commit(rig.owner,p.token,expected,
          {rig.owner.accepted().owner_id,rig.owner.accepted().epoch,p.view.attempt,8,true}).status,PS::StaleTrial);
      }
    }
    EXPECT_EQ(Bytes(output),saved);Accepted after;ASSERT_TRUE(rig.Read(after));SameAccepted(before,after);
  }
  Candidate retry;ASSERT_TRUE(rig.Begin(retry,Load()));ASSERT_TRUE(rig.Evaluate(retry));ASSERT_TRUE(rig.Commit(retry));
  EXPECT_EQ(rig.owner.accepted().epoch,2u);
}
TEST_F(ConnectorPublication,LateLastSpringFailureRollsBackAllMaterialsAndRetriesExactly) {
  Rig rig,clean;ASSERT_TRUE(rig.Start());ASSERT_TRUE(clean.Start());
  for(auto* r:{&rig,&clean}) {Candidate first;ASSERT_TRUE(r->Begin(first,Load()));ASSERT_TRUE(r->Evaluate(first));ASSERT_TRUE(r->Commit(first));}
  Accepted before;ASSERT_TRUE(rig.Read(before));Candidate failed;ASSERT_TRUE(rig.Begin(failed,Load()));
  ASSERT_EQ(rig.qb.EvaluateCandidate(rig.owner,failed.token,failed.view,&failed.qd).status,q::BatchStatus::Success);
  ASSERT_EQ(rig.tb.EvaluateCandidate(rig.owner,failed.token,failed.view,&failed.td).status,t::BatchStatus::Success);
  const auto* x=failed.view.kinematics.position_xyz;
  const auto& last=rig.connectors.connections()[rig.connectors.connection_count()-1];
  ASSERT_EQ(cudaMemcpyAsync(const_cast<double*>(x)+3*last.global_node[1],x+3*last.global_node[0],
      3*sizeof(double),cudaMemcpyDeviceToDevice,failed.view.stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(failed.view.stream),cudaSuccess);
  auto output=before.common;const auto saved=Bytes(output);
  const auto cr=rig.connector.EvaluateCandidate(rig.owner,failed.token,failed.view,&failed.cd);
  EXPECT_EQ(cr.status,CS::ElementFailure)<<cr.message;EXPECT_EQ(cr.element,1u);
  EXPECT_NE(rig.publication.Prepare(rig.owner,failed.token,failed.qd,failed.td,failed.cd,&output).status,PS::Success);
  EXPECT_EQ(Bytes(output),saved);Accepted held;ASSERT_TRUE(rig.Read(held));SameAccepted(before,held);
  Candidate retry,truth;ASSERT_TRUE(rig.Begin(retry,Load()));ASSERT_TRUE(clean.Begin(truth,Load()));
  ASSERT_TRUE(rig.Evaluate(retry));ASSERT_TRUE(clean.Evaluate(truth));ASSERT_TRUE(rig.Commit(retry));ASSERT_TRUE(clean.Commit(truth));
  Accepted a,b;ASSERT_TRUE(rig.Read(a));ASSERT_TRUE(clean.Read(b));SameMotion(a.nodes,b.nodes);
  for(unsigned e=0;e<2;++e) {
    EXPECT_EQ(type25_test::EvaluationValues(a.connectors[e]),type25_test::EvaluationValues(b.connectors[e]));
    EXPECT_EQ(a.connectors[e].history.active,b.connectors[e].history.active);
  }
  EXPECT_EQ(Bytes(a.common.kinetic),Bytes(b.common.kinetic));
}
TEST_F(ConnectorPublication,DuplicateClaimsAndCudaFailureCannotPartiallyPublish) {
  Rig rig;ASSERT_TRUE(rig.Start());fe::ShellBatchPublication duplicate;
  EXPECT_NE(duplicate.Initialize(rig.owner,rig.qb,rig.tb,rig.connector).status,PS::Success);
  EXPECT_EQ(duplicate.allocations().device_bytes,0u);
  Candidate p;ASSERT_TRUE(rig.Begin(p,Load()));ASSERT_TRUE(rig.Evaluate(p));
  const auto stamp=rig.owner.accepted();
  temporal::Noop<<<1,0,0,p.view.stream>>>();ASSERT_NE(cudaPeekAtLastError(),cudaSuccess);
  const auto r=rig.publication.Commit(rig.owner,p.token,p.common,
      {stamp.owner_id,stamp.epoch,p.view.attempt,8,true});
  EXPECT_EQ(r.status,PS::NodalFailure);EXPECT_EQ(r.nodal_status,fe::NodalStatus::DeviceFailure);
  temporal::SameStamp(stamp,rig.owner.accepted());
  spring::BatchDiagnostics saved=p.cd;const auto bytes=Bytes(saved);
  EXPECT_NE(rig.connector.CopyAcceptedDiagnostics(stamp,&saved).status,CS::Success);
  EXPECT_EQ(Bytes(saved),bytes);
}
} // namespace nodal_mass_test::joined_connector
