#include "RigidShellContactFixture.h"
namespace {
using namespace rigid_shell_contact_test;
TEST_F(Cuda, GroupedStartupRejectsStandalonePrescribedAndPartialScopeBeforeElementReads) {
  Fixture f; ASSERT_TRUE(f.Prepare()); fe::FENodalState owner; ASSERT_EQ(f.Owner(owner).status,fe::NodalStatus::Ok);
  q::QephBatch qb; t::T3Batch tb; auto qc=f.QConfig(owner.accepted()); auto tc=f.TConfig(owner.accepted());
  EXPECT_EQ(qb.Initialize(qc,reinterpret_cast<const q::QephBatchElement*>(1)).status,q::BatchStatus::InvalidInput);
  EXPECT_EQ(tb.Initialize(tc,reinterpret_cast<const t::T3BatchElement*>(1)).status,t::BatchStatus::InvalidInput);
  qc.usage=q::BatchUsage::PrescribedFields; tc.usage=t::BatchUsage::PrescribedFields;
  EXPECT_EQ(qb.InitializeJoined(qc,f.binding).status,q::BatchStatus::InvalidInput);
  EXPECT_EQ(tb.InitializeJoined(tc,f.binding).status,t::BatchStatus::InvalidInput);
  qc=f.QConfig(owner.accepted()); tc=f.TConfig(owner.accepted());
  qc.owner.rigid_groups.source_instance_id=0; tc.owner.rigid_groups.member_count=0;
  EXPECT_EQ(qb.InitializeJoined(qc,f.binding).status,q::BatchStatus::InvalidInput);
  EXPECT_EQ(tb.InitializeJoined(tc,f.binding).status,t::BatchStatus::InvalidInput);
  sc::NodalWallContactDevice wall; auto wc=f.WallConfig(owner.accepted()); wc.owner.rigid_groups.group_count=0;
  EXPECT_EQ(wall.Initialize(wc,f.Wall(),f.weights,{reinterpret_cast<const double*>(1),Nodes,3,1},
      reinterpret_cast<const double*>(1),reinterpret_cast<const std::uint8_t*>(1),f.motion).status,
      sc::NodalWallDeviceStatus::InvalidInput);
}
TEST_F(Cuda, RestAndMovingGroupsRequireAuthenticatedAcceptedSourcesAndPreserveRejectedLoads) {
  for(double speed:{0.,.25}) {
    Rig r; ASSERT_TRUE(r.Initialize(Source,speed,false)); Prepared p;
    ASSERT_EQ(r.owner.BeginTrial(&p.token,&p.assembly).status,fe::NodalStatus::Ok);
    const auto loads=ReadLoads(p.assembly); sc::NodalWallDiagnostics output; output.attempt=981;
    const auto bytes=Bytes(output);
    EXPECT_EQ(r.qeph.AssembleAccepted(p.assembly).status,q::BatchStatus::InvalidInput);
    EXPECT_EQ(r.t3.AssembleAccepted(p.assembly).status,t::BatchStatus::InvalidInput);
    EXPECT_EQ(r.contact.AssembleAccepted(p.assembly,&output).status,sc::NodalWallDeviceStatus::InvalidInput);
    auto bad=p.assembly; bad.mass.inverse_mass=reinterpret_cast<const double*>(1);
    EXPECT_EQ(r.qeph.AssembleAccepted(r.owner,bad).status,q::BatchStatus::StaleTrial);
    EXPECT_EQ(r.t3.AssembleAccepted(r.owner,bad).status,t::BatchStatus::StaleTrial);
    EXPECT_EQ(r.contact.AssembleAccepted(r.owner,bad,&output).status,sc::NodalWallDeviceStatus::StaleAttempt);
    bad=p.assembly; bad.rigid_groups.member_count--;
    EXPECT_EQ(r.qeph.AssembleAccepted(r.owner,bad).status,q::BatchStatus::StaleTrial);
    EXPECT_EQ(r.t3.AssembleAccepted(r.owner,bad).status,t::BatchStatus::StaleTrial);
    EXPECT_EQ(r.contact.AssembleAccepted(r.owner,bad,&output).status,sc::NodalWallDeviceStatus::StaleAttempt);
    EXPECT_EQ(Bytes(output),bytes); EXPECT_EQ(ReadLoads(p.assembly),loads);
    EXPECT_EQ(r.qeph.AssembleAccepted(r.owner,p.assembly).status,q::BatchStatus::Success);
    EXPECT_EQ(r.t3.AssembleAccepted(r.owner,p.assembly).status,t::BatchStatus::Success);
    EXPECT_EQ(r.contact.AssembleAccepted(r.owner,p.assembly,&output).status,sc::NodalWallDeviceStatus::Ok);
    EXPECT_EQ(p.assembly.mass.model,sc::TranslationMassModel::kUnspecified);
    r.Discard(); ASSERT_TRUE(r.Bind());
  }
}
TEST_F(Cuda, GroupedNativeMassInertiaFixityAndInitialReferenceChecksRemainClosed) {
  for(unsigned defect=0;defect<4;++defect) {
    SCOPED_TRACE(defect); Fixture f; ASSERT_TRUE(f.Prepare());
    // Free nodes2/3 are outside the group: the owner admits their supplied
    // coefficients, while each shell must still match the complete native union.
    if(defect==0) f.initial.inverse[2]*=1.01;
    if(defect==1) f.initial.inverse_inertia[3]*=1.01;
    if(defect==2) f.initial.fixed[2]=1;
    if(defect==3) f.initial.x[3*3+2]+=.001;
    fe::FENodalState owner; ASSERT_EQ(f.Owner(owner).status,fe::NodalStatus::Ok);
    q::QephBatch qb; t::T3Batch tb;
    ASSERT_EQ(qb.InitializeJoined(f.QConfig(owner.accepted()),f.binding).status,q::BatchStatus::Success);
    ASSERT_EQ(tb.InitializeJoined(f.TConfig(owner.accepted()),f.binding).status,t::BatchStatus::Success);
    fe::NodalTrialToken token; fe::NodalAssemblyView a;
    ASSERT_EQ(owner.BeginTrial(&token,&a).status,fe::NodalStatus::Ok);
    EXPECT_EQ(qb.AssembleAccepted(owner,a).status,defect==3?q::BatchStatus::InvalidInput:q::BatchStatus::InvalidMass);
    owner.Discard(); qb.DiscardTrial();
    ASSERT_EQ(owner.BeginTrial(&token,&a).status,fe::NodalStatus::Ok);
    EXPECT_EQ(tb.AssembleAccepted(owner,a).status,defect==3?t::BatchStatus::InvalidInput:t::BatchStatus::InvalidMass);
    EXPECT_EQ(owner.accepted().epoch,0u);
  }
}
TEST_F(Cuda, SameCountOtherQualifiedGroupSourceCannotReplaceConfiguredOwner) {
  Rig r,other; ASSERT_TRUE(r.Initialize(Source,0,false)); ASSERT_TRUE(other.Initialize(Source+1,0,false));
  ASSERT_EQ(r.owner.rigid_groups().group_count,other.owner.rigid_groups().group_count);
  ASSERT_EQ(r.owner.rigid_groups().member_count,other.owner.rigid_groups().member_count);
  Prepared p; ASSERT_EQ(r.owner.BeginTrial(&p.token,&p.assembly).status,fe::NodalStatus::Ok);
  auto forged=p.assembly; forged.rigid_groups=other.owner.rigid_groups();
  sc::NodalWallDiagnostics d;
  EXPECT_EQ(r.qeph.AssembleAccepted(r.owner,forged).status,q::BatchStatus::StaleTrial);
  EXPECT_EQ(r.t3.AssembleAccepted(r.owner,forged).status,t::BatchStatus::StaleTrial);
  EXPECT_EQ(r.contact.AssembleAccepted(r.owner,forged,&d).status,sc::NodalWallDeviceStatus::StaleAttempt);
  // Even agreement between a fabricated config and view is insufficient.
  auto stamp=r.owner.accepted(); stamp.rigid_groups=other.owner.rigid_groups();
  q::QephBatch qb; t::T3Batch tb; sc::NodalWallContactDevice wall;
  ASSERT_EQ(qb.InitializeJoined(r.source.QConfig(stamp),r.source.binding).status,q::BatchStatus::Success);
  ASSERT_EQ(tb.InitializeJoined(r.source.TConfig(stamp),r.source.binding).status,t::BatchStatus::Success);
  ASSERT_EQ(wall.Initialize(r.source.WallConfig(stamp),r.source.Wall(),r.source.weights,
      {r.source.initial.x.data(),Nodes,3,1},r.source.initial.inverse.data(),r.source.initial.fixed.data(),r.source.motion).status,sc::NodalWallDeviceStatus::Ok);
  EXPECT_EQ(qb.AssembleAccepted(r.owner,forged).status,q::BatchStatus::StaleTrial);
  EXPECT_EQ(tb.AssembleAccepted(r.owner,forged).status,t::BatchStatus::StaleTrial);
  EXPECT_EQ(wall.AssembleAccepted(r.owner,forged,&d).status,sc::NodalWallDeviceStatus::StaleAttempt);
  EXPECT_EQ(ReadLoads(p.assembly),(std::array<double,6*Nodes>{}));
  r.Discard(); ASSERT_TRUE(r.Bind());
}
TEST_F(Cuda, GroupedCandidatesRequireActualCommonTokenAndAtomicDiagnosticOutputs) {
  Rig r,other; ASSERT_TRUE(r.Initialize()); ASSERT_TRUE(other.Initialize(Source+1));
  Prepared p,o; ASSERT_TRUE(Assemble(r,p)); ASSERT_TRUE(Advance(r,p));
  ASSERT_TRUE(Assemble(other,o)); ASSERT_TRUE(Advance(other,o));
  q::BatchDiagnostics qd; t::BatchDiagnostics td; sc::NodalWallDiagnostics wd;
  qd.attempt=td.attempt=wd.attempt=988;
  const auto qb=Bytes(qd); const auto tb=Bytes(td); const auto wb=Bytes(wd);
  EXPECT_EQ(r.qeph.EvaluateCandidate(p.view,&qd).status,q::BatchStatus::InvalidInput);
  EXPECT_EQ(r.t3.EvaluateCandidate(p.view,&td).status,t::BatchStatus::InvalidInput);
  EXPECT_EQ(r.contact.EvaluateCandidate(p.view,&wd).status,sc::NodalWallDeviceStatus::InvalidInput);
  auto forged=p.view; forged.kinematics=o.view.kinematics;
  EXPECT_EQ(r.qeph.EvaluateCandidate(r.owner,p.token,forged,&qd).status,q::BatchStatus::StaleTrial);
  EXPECT_EQ(r.t3.EvaluateCandidate(r.owner,p.token,forged,&td).status,t::BatchStatus::StaleTrial);
  EXPECT_EQ(r.contact.EvaluateCandidate(r.owner,p.token,forged,&wd).status,sc::NodalWallDeviceStatus::StaleAttempt);
  const auto token=Bytes(p.token);
  EXPECT_EQ(r.qeph.EvaluateCandidate(r.owner,p.token,p.view,reinterpret_cast<q::BatchDiagnostics*>(&p.token)).status,q::BatchStatus::InvalidInput);
  EXPECT_EQ(r.t3.EvaluateCandidate(r.owner,p.token,p.view,reinterpret_cast<t::BatchDiagnostics*>(&p.token)).status,t::BatchStatus::InvalidInput);
  EXPECT_EQ(r.contact.EvaluateCandidate(r.owner,p.token,p.view,reinterpret_cast<sc::NodalWallDiagnostics*>(&p.token)).status,sc::NodalWallDeviceStatus::InvalidInput);
  EXPECT_EQ(Bytes(p.token),token);
  EXPECT_EQ(r.qeph.EvaluateCandidate(r.owner,o.token,p.view,&qd).status,q::BatchStatus::StaleTrial);
  EXPECT_EQ(r.t3.EvaluateCandidate(r.owner,o.token,p.view,&td).status,t::BatchStatus::StaleTrial);
  EXPECT_EQ(r.contact.EvaluateCandidate(r.owner,o.token,p.view,&wd).status,sc::NodalWallDeviceStatus::StaleAttempt);
  EXPECT_EQ(Bytes(qd),qb); EXPECT_EQ(Bytes(td),tb); EXPECT_EQ(Bytes(wd),wb);
  // BorrowPrepared deliberately invalidates a trial when given a foreign token.
  // Retry through a fresh common transaction, preserving the accepted phase.
  r.Discard(); Prepared retry; ASSERT_TRUE(Prepare(r,retry)); ASSERT_TRUE(Commit(r,retry));
}
} // namespace
