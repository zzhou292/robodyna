#include "MixedShellLedger.h"

namespace mixed_shell_test {
namespace {
using S=fe::ShellPublicationStatus;
q::QephBatchConfig QConfig(const Rig& r) {
  q::QephBatchConfig c; c.owner=r.owner.accepted(); c.element_count=1;
  c.configuration_id=Configuration; c.qualification_id=Qualification; c.usage=q::BatchUsage::PrescribedFields; return c;
}
t::T3BatchConfig TConfig(const Rig& r) {
  t::T3BatchConfig c; c.owner=r.owner.accepted(); c.element_count=1;
  c.configuration_id=Configuration; c.qualification_id=Qualification; c.usage=t::BatchUsage::PrescribedFields; return c;
}
void Preserved(Rig& r,const Snapshot& state,const Staged& elements,const fe::ShellBatchDiagnostics& common) {
  Snapshot now; ASSERT_TRUE(Read(r.owner,now)); SameState(state,now);
  Staged saved; ASSERT_TRUE(Accepted(r,saved)); ExactResults(elements,saved);
  EXPECT_EQ(Bytes(elements.diagnostics.qeph),Bytes(saved.diagnostics.qeph));
  EXPECT_EQ(Bytes(elements.diagnostics.t3),Bytes(saved.diagnostics.t3));
  fe::ShellBatchDiagnostics diagnostic;
  ASSERT_EQ(r.publication.CopyAcceptedDiagnostics(r.owner.accepted(),&diagnostic).status,S::Success);
  EXPECT_EQ(Bytes(diagnostic),Bytes(common));
}
bool TypedCandidates(Rig& r,const fe::NodalPreparedView& p,q::BatchDiagnostics& qd,t::BatchDiagnostics& td) {
  const auto qr=r.qeph.EvaluateCandidate(p,&qd); const auto tr=r.t3.EvaluateCandidate(p,&td);
  EXPECT_EQ(qr.status,q::BatchStatus::Success)<<qr.message; EXPECT_EQ(tr.status,t::BatchStatus::Success)<<tr.message;
  return qr.status==q::BatchStatus::Success&&tr.status==t::BatchStatus::Success;
}
static __global__ void SetPosition(double* x,unsigned node,double a,double b,double c) {
  x[3*node]=a; x[3*node+1]=b; x[3*node+2]=c;
}
} // namespace

TEST_F(MixedShellCuda,JoinedStartupRequiresCompleteUnionMassRestAndCommonImmutableScope) {
  Rig base; ASSERT_TRUE(base.Initialize());
  for(unsigned kind=0;kind<4;++kind) {
    SCOPED_TRACE(kind);
    q::QephBatch qbad; t::T3Batch tbad; auto qc=QConfig(base); auto tc=TConfig(base);
    if(kind==0) { qc.element_count=2; tc.element_count=2; }
    if(kind==1) { qc.owner.node_count=4; tc.owner.node_count=4; }
    if(kind==2) { qc.usage=q::BatchUsage::CoupledForces; tc.usage=t::BatchUsage::CoupledForces; }
    if(kind==3) { qc.max_device_bytes=1; tc.max_device_bytes=1; }
    EXPECT_NE(qbad.InitializeJoined(qc,base.binding).status,q::BatchStatus::Success);
    EXPECT_NE(tbad.InitializeJoined(tc,base.binding).status,t::BatchStatus::Success);
    EXPECT_EQ(qbad.allocations().device_allocations,0u); EXPECT_EQ(tbad.allocations().device_allocations,0u);
    ASSERT_EQ(qbad.InitializeJoined(QConfig(base),base.binding).status,q::BatchStatus::Success);
    ASSERT_EQ(tbad.InitializeJoined(TConfig(base),base.binding).status,t::BatchStatus::Success);
  }
  for(unsigned kind=0;kind<5;++kind) {
    SCOPED_TRACE(kind);
    Rig bad; ASSERT_TRUE(bad.PrepareReference());
    if(kind==0) bad.initial.inverse[4]*=2; // Node absent from Q4 must still be checked by Q4.
    if(kind==1) bad.initial.inverse_inertia[0]*=2; // Node absent from T3 must still be checked by T3.
    if(kind==2) bad.initial.x[0]+=.125;
    if(kind==3) bad.initial.v[3*4]=.125;
    if(kind==4) { bad.initial.fixed[4]=7; bad.initial.rotation_fixed[4]=1; bad.initial.inverse[4]=0; bad.initial.inverse_inertia[4]=0; }
    ASSERT_EQ(bad.initial.Initialize(bad.owner).status,fe::NodalStatus::Ok);
    ASSERT_TRUE(bad.InitializeParticipants()); Snapshot before; ASSERT_TRUE(Read(bad.owner,before));
    // Each family gets a fresh attempt, so one sticky failure cannot stand in
    // for independently checking the other's full union mass/rest validation.
    for(unsigned family=0;family<2;++family) {
      fe::NodalTrialToken token; fe::NodalAssemblyView view;
      ASSERT_EQ(bad.owner.BeginTrial(&token,&view).status,fe::NodalStatus::Ok);
      if(family==0) EXPECT_EQ(bad.qeph.AssembleAccepted(view).status,
          kind<2||kind==4?q::BatchStatus::InvalidMass:q::BatchStatus::InvalidInput);
      else EXPECT_EQ(bad.t3.AssembleAccepted(view).status,
          kind<2||kind==4?t::BatchStatus::InvalidMass:t::BatchStatus::InvalidInput);
      EXPECT_EQ(bad.owner.SealAssembly(token).status,fe::NodalStatus::ContributorFailure); bad.Discard();
    }
    Snapshot after; ASSERT_TRUE(Read(bad.owner,after)); SameState(before,after);
    EXPECT_NE(bad.publication.Initialize(bad.owner,bad.qeph,bad.t3).status,S::Success);
    EXPECT_EQ(bad.publication.allocations().device_allocations,0u);
  }
  // Different E leaves native mass/rest identical, but must not be mistaken
  // for the same exact physical inventory. ID/qualification differences also
  // fail before the coordinator allocates or publishes its zero diagnostics.
  for(unsigned kind=0;kind<3;++kind) {
    SCOPED_TRACE(kind);
    Rig r; ASSERT_TRUE(r.Initialize()); auto in=r.input;
    if(kind==0) in.t3.young_modulus*=2;
    fe::ShellBatchBinding other; ASSERT_EQ(other.Initialize(in).status,fe::ShellBindingStatus::Success);
    t::T3Batch alternate; auto tc=TConfig(r);
    if(kind==1) ++tc.configuration_id;
    if(kind==2) ++tc.qualification_id;
    ASSERT_EQ(alternate.InitializeJoined(tc,other).status,t::BatchStatus::Success);
    fe::NodalTrialToken token; fe::NodalAssemblyView view;
    ASSERT_EQ(r.owner.BeginTrial(&token,&view).status,fe::NodalStatus::Ok);
    ASSERT_EQ(r.qeph.AssembleAccepted(view).status,q::BatchStatus::Success);
    ASSERT_EQ(alternate.AssembleAccepted(view).status,t::BatchStatus::Success);
    r.owner.Discard(); r.qeph.DiscardTrial(); alternate.DiscardTrial();
    fe::ShellBatchPublication rejected;
    EXPECT_EQ(rejected.Initialize(r.owner,r.qeph,alternate).status,S::InvalidInput);
    EXPECT_EQ(rejected.allocations().device_allocations,0u); EXPECT_EQ(r.owner.accepted().epoch,0u);
  }
  // Plausible owner stamps and numerically valid foreign buffers can satisfy
  // the raw assembly operation. They cannot authenticate the initial binding
  // or its zero kinetic cache. Check either family separately and both at once.
  for(unsigned kind=0;kind<4;++kind) {
    SCOPED_TRACE(kind);
    Rig r,foreign; ASSERT_TRUE(r.PrepareReference()); ASSERT_TRUE(foreign.Initialize());
    if(kind==3) r.initial.inverse[4]*=2; // Forged mass would hide actual wrong m.
    ASSERT_EQ(r.initial.Initialize(r.owner).status,fe::NodalStatus::Ok);
    ASSERT_TRUE(r.InitializeParticipants());
    Snapshot before; ASSERT_TRUE(Read(r.owner,before));
    fe::NodalTrialToken token,foreign_token; fe::NodalAssemblyView view,foreign_view;
    ASSERT_EQ(r.owner.BeginTrial(&token,&view).status,fe::NodalStatus::Ok);
    ASSERT_EQ(foreign.owner.BeginTrial(&foreign_token,&foreign_view).status,fe::NodalStatus::Ok);
    auto forged=view;
    if(kind!=1) {
      forged.mass=foreign_view.mass; forged.inverse_inertia=foreign_view.inverse_inertia;
      forged.translation_fixed_bits=foreign_view.translation_fixed_bits;
      forged.rotation_fixed=foreign_view.rotation_fixed;
    }
    if(kind==1||kind==2) forged.accepted=foreign_view.accepted;
    const auto& qview=kind==1?view:forged;
    const auto& tview=kind==0?view:forged;
    ASSERT_EQ(r.qeph.AssembleAccepted(qview).status,q::BatchStatus::Success);
    ASSERT_EQ(r.t3.AssembleAccepted(tview).status,t::BatchStatus::Success);
    r.Discard(); foreign.Discard();
    Staged cached; ASSERT_TRUE(Accepted(r,cached));
    fe::ShellBatchDiagnostics output; output.valid=true; output.kinetic.translation=17;
    const auto held=Bytes(output);
    EXPECT_EQ(r.publication.Initialize(r.owner,r.qeph,r.t3).status,S::StaleTrial);
    EXPECT_EQ(r.publication.allocations().device_allocations,0u);
    EXPECT_EQ(r.publication.CopyAcceptedDiagnostics(r.owner.accepted(),&output).status,S::NotInitialized);
    EXPECT_EQ(Bytes(output),held);
    // A later genuine raw assembly does not overwrite the first binding
    // record. Recovery needs fresh participants, not a replacement provenance.
    if(kind!=3) {
      ASSERT_EQ(r.owner.BeginTrial(&token,&view).status,fe::NodalStatus::Ok);
      ASSERT_EQ(r.qeph.AssembleAccepted(view).status,q::BatchStatus::Success);
      ASSERT_EQ(r.t3.AssembleAccepted(view).status,t::BatchStatus::Success);
      r.Discard();
    }
    EXPECT_EQ(r.publication.Initialize(r.owner,r.qeph,r.t3).status,S::StaleTrial);
    EXPECT_EQ(r.publication.allocations().device_allocations,0u);
    Snapshot after; ASSERT_TRUE(Read(r.owner,after)); SameState(before,after);
    Staged still; ASSERT_TRUE(Accepted(r,still)); ExactResults(cached,still);
  }
  Rig clean_retry; ASSERT_TRUE(clean_retry.Initialize()); ASSERT_TRUE(clean_retry.Bind());
  EXPECT_EQ(clean_retry.owner.accepted().epoch,0u);
}

TEST_F(MixedShellCuda,MissingForeignAndOutputPreflightFailuresLeaveAllAcceptedOutputsIntact) {
  Rig r,foreign; ASSERT_TRUE(r.Initialize()); ASSERT_TRUE(r.Bind());
  ASSERT_TRUE(foreign.Initialize()); ASSERT_TRUE(foreign.Bind());
  Snapshot initial; ASSERT_TRUE(Read(r.owner,initial)); Staged cache; ASSERT_TRUE(Accepted(r,cache));
  fe::ShellBatchDiagnostics accepted;
  ASSERT_EQ(r.publication.CopyAcceptedDiagnostics(r.owner.accepted(),&accepted).status,S::Success);
  Prepared p,other; ASSERT_TRUE(Prepare(foreign,Schedule(foreign,0),other));
  q::BatchDiagnostics qd; t::BatchDiagnostics td;
  ASSERT_TRUE(Prepare(r,Schedule(r,0),p));
  const auto qbytes=Bytes(qd); const auto tbytes=Bytes(td);
  EXPECT_EQ(r.qeph.EvaluateCandidate(other.view,&qd).status,q::BatchStatus::WrongOwner);
  EXPECT_EQ(r.t3.EvaluateCandidate(other.view,&td).status,t::BatchStatus::WrongOwner);
  EXPECT_EQ(Bytes(qd),qbytes); EXPECT_EQ(Bytes(td),tbytes);
  ASSERT_NE(p.view.stream,other.view.stream);
  auto wrong=p.view; wrong.stream=other.view.stream;
  EXPECT_EQ(r.qeph.EvaluateCandidate(wrong,&qd).status,q::BatchStatus::StaleTrial);
  EXPECT_EQ(r.t3.EvaluateCandidate(wrong,&td).status,t::BatchStatus::StaleTrial);
  // Only one required material result exists: a plausible receipt cannot
  // authorize measurement or publication of a complete mixed interval.
  ASSERT_EQ(r.qeph.EvaluateCandidate(p.view,&qd).status,q::BatchStatus::Success);
  fe::ShellBatchDiagnostics output=accepted; const auto held=Bytes(output);
  EXPECT_EQ(r.publication.Prepare(r.owner,p.token,qd,td,&output).status,S::StaleTrial);
  EXPECT_EQ(Bytes(output),held);
  ASSERT_NO_FATAL_FAILURE(Preserved(r,initial,cache,accepted));
  // Matching numeric stamps with different actual buffers fail BEFORE the
  // kinetic kernel; both typed operations intentionally lack token authority.
  ASSERT_TRUE(Prepare(r,Schedule(r,0),p)); auto forged=p.view;
  forged.kinematics=other.view.kinematics; forged.base_kinematics=other.view.base_kinematics;
  ASSERT_TRUE(TypedCandidates(r,forged,qd,td));
  EXPECT_EQ(r.publication.Prepare(r.owner,p.token,qd,td,&output).status,S::StaleTrial);
  EXPECT_EQ(Bytes(output),held);
  ASSERT_NO_FATAL_FAILURE(Preserved(r,initial,cache,accepted));
  for(unsigned kind=0;kind<2;++kind) {
    ASSERT_TRUE(Prepare(r,Schedule(r,0),p)); ASSERT_TRUE(TypedCandidates(r,p.view,qd,td));
    const auto qsaved=Bytes(qd); const auto tsaved=Bytes(td);
    auto* destination=kind?reinterpret_cast<fe::ShellBatchDiagnostics*>(&qd):nullptr;
    EXPECT_EQ(r.publication.Prepare(r.owner,p.token,qd,td,destination).status,S::InvalidInput);
    EXPECT_EQ(Bytes(qd),qsaved); EXPECT_EQ(Bytes(td),tsaved);
    ASSERT_NO_FATAL_FAILURE(Preserved(r,initial,cache,accepted));
  }
  foreign.Discard();
  ASSERT_TRUE(Prepare(r,Schedule(r,0),p)); Staged complete; ASSERT_TRUE(Evaluate(r,p,complete));
  NativePair native; NativeTrials truth; ASSERT_TRUE(native.Initialize(r));
  ASSERT_TRUE(native.Check(r,p,complete,truth)); CheckLedgers(r,initial,cache,p,complete);
  ASSERT_TRUE(Publish(r,p,complete)); EXPECT_EQ(r.owner.accepted().epoch,1u);
  RecordProperty("native_qeph_intervals",1); RecordProperty("native_t3_intervals",1);
}

TEST_F(MixedShellCuda,StandaloneAndTamperedJointReceiptsCannotPublishOneFamily) {
  Rig r; ASSERT_TRUE(r.Initialize()); ASSERT_TRUE(r.Bind());
  fe::ShellBatchPublication duplicate;
  EXPECT_EQ(duplicate.Initialize(r.owner,r.qeph,r.t3).status,S::InvalidInput);
  EXPECT_EQ(duplicate.allocations().device_allocations,0u);
  Snapshot before; ASSERT_TRUE(Read(r.owner,before)); Staged cache; ASSERT_TRUE(Accepted(r,cache));
  fe::ShellBatchDiagnostics accepted;
  ASSERT_EQ(r.publication.CopyAcceptedDiagnostics(r.owner.accepted(),&accepted).status,S::Success);
  for(unsigned kind=0;kind<9;++kind) {
    SCOPED_TRACE(kind);
    Prepared p; Staged candidate; ASSERT_TRUE(Prepare(r,Schedule(r,0),p)); ASSERT_TRUE(Evaluate(r,p,candidate));
    auto expected=candidate.diagnostics; auto receipt=Receipt(candidate);
    if(kind==0) {
      EXPECT_EQ(q::CommitQephTrial(r.owner,p.token,r.qeph,expected.qeph,receipt).status,q::BatchStatus::InvalidInput);
      r.Discard();
    } else if(kind==1) {
      EXPECT_EQ(t::CommitT3Trial(r.owner,p.token,r.t3,expected.t3,receipt).status,t::BatchStatus::InvalidInput);
      r.Discard();
    } else {
      if(kind==2) receipt.passed=false;
      if(kind==3) ++receipt.qualification_id;
      if(kind==4) ++expected.t3.attempt;
      if(kind==5) expected.qeph.kinetic_available=true;
      if(kind==6) expected.kinetic.translation=std::nextafter(expected.kinetic.translation,1.);
      if(kind==7) expected.t3.internal_work[0]=std::nextafter(expected.t3.internal_work[0],1.);
      if(kind==8) r.t3.DiscardTrial(); // Previously measured candidate is no longer jointly pending.
      EXPECT_EQ(r.publication.Commit(r.owner,p.token,expected,receipt).status,S::StaleTrial);
    }
    ASSERT_NO_FATAL_FAILURE(Preserved(r,before,cache,accepted));
    EXPECT_EQ(r.owner.Commit(p.token).status,fe::NodalStatus::WrongPhase);
  }
}

TEST_F(MixedShellCuda,LateFailureInEitherFamilyPreservesNonzeroHistoryAndCleanRetry) {
  Rig r,clean; ASSERT_TRUE(r.Initialize()); ASSERT_TRUE(r.Bind());
  ASSERT_TRUE(clean.Initialize()); ASSERT_TRUE(clean.Bind());
  NativePair native,clean_native; ASSERT_TRUE(native.Initialize(r)); ASSERT_TRUE(clean_native.Initialize(clean));
  for(unsigned side=0;side<2;++side) {
    auto& current=side?clean:r; auto& oracle=side?clean_native:native;
    Snapshot base; Staged cache,next; Prepared p; NativeTrials truth;
    ASSERT_TRUE(Read(current.owner,base)); ASSERT_TRUE(Accepted(current,cache));
    ASSERT_TRUE(Prepare(current,Schedule(current,0),p)); ASSERT_TRUE(Evaluate(current,p,next));
    ASSERT_TRUE(oracle.Check(current,p,next,truth)); CheckLedgers(current,base,cache,p,next);
    ASSERT_TRUE(Publish(current,p,next)); oracle.Accept(truth);
  }
  Snapshot before; ASSERT_TRUE(Read(r.owner,before)); Staged cache; ASSERT_TRUE(Accepted(r,cache));
  fe::ShellBatchDiagnostics accepted;
  ASSERT_EQ(r.publication.CopyAcceptedDiagnostics(r.owner.accepted(),&accepted).status,S::Success);
  ASSERT_GT(std::abs(cache.diagnostics.qeph.internal_work[0]),1e-12);
  ASSERT_GT(std::abs(cache.diagnostics.t3.internal_work[0]),1e-12);
  for(unsigned t3_first=0;t3_first<2;++t3_first) {
    SCOPED_TRACE(t3_first);
    Prepared p; ASSERT_TRUE(Prepare(r,Schedule(r,1),p));
    auto qd=cache.diagnostics.qeph; auto td=cache.diagnostics.t3;
    if(!t3_first) {
      ASSERT_EQ(r.qeph.EvaluateCandidate(p.view,&qd).status,q::BatchStatus::Success);
      const auto held=Bytes(td); const auto& x=p.endpoint.x;
      SetPosition<<<1,1,0,p.view.stream>>>(const_cast<double*>(p.view.kinematics.position_xyz),4,x[3],x[4],x[5]);
      ASSERT_EQ(cudaStreamSynchronize(p.view.stream),cudaSuccess);
      EXPECT_EQ(r.t3.EvaluateCandidate(p.view,&td).status,t::BatchStatus::ElementFailure); EXPECT_EQ(Bytes(td),held);
    } else {
      ASSERT_EQ(r.t3.EvaluateCandidate(p.view,&td).status,t::BatchStatus::Success);
      const auto held=Bytes(qd); const auto& x=p.endpoint.x;
      SetPosition<<<1,1,0,p.view.stream>>>(const_cast<double*>(p.view.kinematics.position_xyz),0,x[3],x[4],x[5]);
      SetPosition<<<1,1,0,p.view.stream>>>(const_cast<double*>(p.view.kinematics.position_xyz),3,x[6],x[7],x[8]);
      ASSERT_EQ(cudaStreamSynchronize(p.view.stream),cudaSuccess);
      EXPECT_EQ(r.qeph.EvaluateCandidate(p.view,&qd).status,q::BatchStatus::ElementFailure); EXPECT_EQ(Bytes(qd),held);
    }
    fe::ShellBatchDiagnostics output=accepted; const auto bytes=Bytes(output);
    EXPECT_EQ(r.publication.Prepare(r.owner,p.token,qd,td,&output).status,S::StaleTrial);
    EXPECT_EQ(Bytes(output),bytes);
    ASSERT_NO_FATAL_FAILURE(Preserved(r,before,cache,accepted));
  }
  Prepared retry,cp; Staged next,truth; NativeTrials native_next,native_clean;
  ASSERT_TRUE(Prepare(r,Schedule(r,1),retry)); ASSERT_TRUE(Evaluate(r,retry,next,true));
  ASSERT_TRUE(Prepare(clean,Schedule(clean,1),cp)); ASSERT_TRUE(Evaluate(clean,cp,truth));
  ASSERT_TRUE(native.Check(r,retry,next,native_next)); ASSERT_TRUE(clean_native.Check(clean,cp,truth,native_clean));
  ExactResults(next,truth); EXPECT_EQ(Bytes(next.diagnostics.kinetic),Bytes(truth.diagnostics.kinetic));
  CheckLedgers(r,before,cache,retry,next);
  Snapshot clean_base; Staged clean_cache; ASSERT_TRUE(Read(clean.owner,clean_base)); ASSERT_TRUE(Accepted(clean,clean_cache));
  CheckLedgers(clean,clean_base,clean_cache,cp,truth);
  ASSERT_TRUE(Publish(r,retry,next)); ASSERT_TRUE(Publish(clean,cp,truth));
  EXPECT_EQ(r.owner.accepted().epoch,2u); EXPECT_EQ(clean.owner.accepted().epoch,2u);
  RecordProperty("native_qeph_intervals",4); RecordProperty("native_t3_intervals",4);
}

TEST_F(MixedShellCuda,PendingCudaFailureBeforeJointCommitPoisonsAllWithoutPublishing) {
  Rig r; ASSERT_TRUE(r.Initialize()); ASSERT_TRUE(r.Bind());
  Prepared p; Staged candidate; ASSERT_TRUE(Prepare(r,Schedule(r,0),p)); ASSERT_TRUE(Evaluate(r,p,candidate));
  const auto stamp=r.owner.accepted(); ASSERT_EQ(cudaPeekAtLastError(),cudaSuccess);
  Noop<<<1,0,0,p.view.stream>>>(); const auto error=cudaPeekAtLastError();
  ASSERT_TRUE(error==cudaErrorInvalidConfiguration||error==cudaErrorInvalidValue);
  const auto report=r.publication.Commit(r.owner,p.token,candidate.diagnostics,Receipt(candidate));
  EXPECT_EQ(report.status,S::NodalFailure); EXPECT_EQ(report.nodal_status,fe::NodalStatus::DeviceFailure);
  SameStamp(r.owner.accepted(),stamp);
  Staged held=candidate; const auto qbytes=Bytes(held.qeph); const auto tbytes=Bytes(held.t3);
  const auto dbytes=Bytes(held.diagnostics);
  EXPECT_EQ(r.qeph.CopyAcceptedResults(stamp,&held.qeph,1,&held.diagnostics.qeph).status,q::BatchStatus::DeviceFailure);
  EXPECT_EQ(r.t3.CopyAcceptedResults(stamp,&held.t3,1,&held.diagnostics.t3).status,t::BatchStatus::DeviceFailure);
  EXPECT_EQ(r.publication.CopyAcceptedDiagnostics(stamp,&held.diagnostics).status,S::DeviceFailure);
  EXPECT_EQ(Bytes(held.qeph),qbytes); EXPECT_EQ(Bytes(held.t3),tbytes); EXPECT_EQ(Bytes(held.diagnostics),dbytes);
}
} // namespace mixed_shell_test
