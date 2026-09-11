#include "Truth.h"
namespace t3_observer_test {
TEST(T3ObserverReduction,FixedTreesPartialBlocksAndIndependentSignedTruth) {
  for(unsigned parents:{1u,127u,128u,129u,263u,33001u}) {
    SCOPED_TRACE(parents);Fixture f(parents);
    const auto before=f.Serial(),actual=f.Staged();
    ASSERT_EQ(actual.status,q::BatchStatus::Success);
    SameExceptSums(actual,before);CheckTruth(f,actual);
    SameControl(f.Staged(),actual);
  }
  Fixture f;
  const auto serial=f.Serial(),actual=f.Staged();
  EXPECT_EQ(serial.diagnostics.internal_work[0],0);
  EXPECT_EQ(actual.diagnostics.internal_work[0],1);
  EXPECT_NE(Bits(actual.diagnostics.internal_work[0]),Bits(serial.diagnostics.internal_work[0]));
}
TEST(T3ObserverReduction,EpochMasksAssemblyBranchAndAllSkinDefaults) {
  for(unsigned epoch=0;epoch<3;++epoch)for(unsigned mask=0;mask<8;++mask)for(bool assembled:{false,true}) {
    Fixture f(Parents,epoch);
    for(unsigned p=0;p<3;++p) {
      auto& r=f.host->slab[1].element[p];auto h=r.proposed_history.data();h.active=mask&(1u<<p)?0:1;
      ASSERT_EQ(q::PrepareFailurePrescribedHistory(f.host->model.element[p].reference,h,
          {(epoch+1)*1e-6,epoch+1},r.proposed_history),q::Status::kSuccess);
    }
    const auto actual=f.Staged(epoch,assembled);
    ASSERT_EQ(actual.status,q::BatchStatus::Success);
    SameExceptSums(actual,f.Serial(epoch,assembled));CheckTruth(f,actual,epoch,assembled);
  }
  Fixture skins(263);
  for(unsigned p=0;p<skins.roles.size();++p) {
    skins.roles[p]=fe::ShellSectionLaw::RigidSkin;
    skins.host->model.element[p]=skins.source.host->model.element[3];
    for(unsigned s=0;s<2;++s)skins.host->slab[s].element[p]=skins.source.host->slab[s].element[3];
  }
  SameControl(skins.Staged(),skins.Serial());
  EXPECT_EQ(skins.Staged().diagnostics.minimum_native_dt,0);
  // No material means these fields are never touched, including signed-zero
  // seeds; a nonfinite untouched seed must still fail the old final check.
  for(bool nonfinite:{false,true}) {
    auto identity=Identity(1);
    identity.internal_work[0]=-0.;
    identity.maximum_absolute_strain=nonfinite?NAN:-0.;
    fe::shell_batch_plasticity_detail::MixedDeviceStorage mixed;mixed.law=skins.roles.data();
    frozen::FinalizeCandidate(skins.host,&skins.host->slab[0],&skins.host->slab[1],skins.View(),identity,&mixed);
    const auto serial=skins.host->control;const auto summary=skins.Reduce();b::Control actual;
    m::FinalizeObservations(*skins.host,skins.host->slab[0],skins.host->slab[1],skins.View(),identity,
        &mixed,summary,actual);
    SameControl(actual,serial);
  }
}
TEST(T3ObserverReduction,EveryFailureReplaysOriginalPriorityPartialFieldsAndRetry) {
  for(unsigned fault=0;fault<10;++fault) {
    SCOPED_TRACE(fault);Fixture f(263);
    if(fault==0) {f.host->candidate_status[260]=q::Status::kInvalidInput;f.host->slab[1].element[1].internal_force[0].x=INFINITY;}
    if(fault==1) {f.host->candidate_status[5]=q::Status::kNonfiniteResult;f.host->candidate_status[260]=q::Status::kInvalidInput;}
    if(fault==2)f.source.fields.orientation[4*(Nodes-1)]=0;
    if(fault==3)f.source.fields.position[3*(Nodes-1)]=INFINITY;
    if(fault==4)f.host->model.element[260].reference.area=-1;
    if(fault==5)f.host->slab[1].element[260].diagnostics.internal_work_increment[0]=INFINITY;
    if(fault==6)f.host->slab[1].element[260].diagnostics.unscaled_element_dt=-0.;
    if(fault==7)f.host->model.joined=false;
    if(fault==9)f.host->model.mapped=false;
    const bool catalog=fault!=8;
    const auto actual=f.Staged(1,true,catalog),serial=f.Serial(1,true,catalog);
    SameControl(actual,serial);
    if(fault!=6 && fault!=9)EXPECT_NE(actual.status,q::BatchStatus::Success);
  }
  Fixture f;
  f.source.fields.orientation[4*(Nodes-1)]=0;
  EXPECT_NE(f.Staged().status,q::BatchStatus::Success);
  f.source.fields.orientation[4*(Nodes-1)]=1;
  ASSERT_EQ(f.Staged().status,q::BatchStatus::Success);CheckTruth(f,f.Staged());
}
TEST(T3ObserverReduction,FinitePrefixProofFallbackPreservesAdmittedAndOverflowPackets) {
  for(bool overflow:{false,true}) {
    Fixture f;
    const double term=overflow?DBL_MAX:DBL_MAX*.375;
    f.host->slab[1].element[0].diagnostics.internal_work_increment[0]=term;
    f.host->slab[1].element[1].diagnostics.internal_work_increment[0]=term;
    f.host->slab[1].element[2].diagnostics.internal_work_increment[0]=-term;
    EXPECT_FALSE(m::FiniteObserverPrefixes(Parents,term));
    const auto actual=f.Staged(),serial=f.Serial();SameControl(actual,serial);
    EXPECT_EQ(actual.status,overflow?q::BatchStatus::NonfiniteResult:q::BatchStatus::Success);
  }
  EXPECT_FALSE(m::FiniteObserverPrefixes(0,0));
  EXPECT_FALSE(m::FiniteObserverPrefixes(std::size_t{UINT32_MAX},0));
  EXPECT_TRUE(m::FiniteObserverPrefixes(UINT32_MAX/4,1));
}
TEST(T3ObserverReduction,SignedZerosGradualUnderflowAndCorruptionControls) {
  Fixture f(263);
  for(unsigned p=0;p<f.roles.size();++p) {
    const double x=p%3==0?std::numeric_limits<double>::denorm_min():p%3==1?-0.:1e-310;
    f.host->slab[1].element[p].diagnostics.internal_work_increment[0]=x;
    if(f.roles[p]==fe::ShellSectionLaw::RigidSkin)f.host->slab[1].element[p].diagnostics.internal_work_increment[0]=-0.;
  }
  const auto actual=f.Staged();ASSERT_EQ(actual.status,q::BatchStatus::Success);
  CheckTruth(f,actual);SameExceptSums(actual,f.Serial());
  const auto truth=Truth(f);const auto bound=truth[2].Bound(f.roles.size(),m::ObserverBlocks(f.roles.size(),Nodes),3);
  EXPECT_GT(Abs(High(actual.diagnostics.internal_work_increment[0])+High(1e-305)-truth[2].value),bound);
}
TEST(T3ObserverReduction,CompleteTypedTailCapAndLegacyLayout) {
  b::Layout full,exact,short_cap,legacy;
  ASSERT_TRUE(full.InitializeMapped(21301,372435,std::size_t{2}<<30));
  EXPECT_EQ(full.assembly.observer.count,256u);
  EXPECT_EQ(full.assembly.observer.bytes,32768u);
  const auto previous=sizeof(b::Storage)-sizeof(void*)+(sizeof(q::T3BatchElement)+2*sizeof(q::ForceTrial)+sizeof(q::Status))*21301+
      (sizeof(q::Vec3)+4*sizeof(double))*372435+29753540;
  EXPECT_EQ(full.bytes,previous+32768u+sizeof(void*));
  RecordProperty("full_mapped_arena_bytes",std::to_string(full.bytes));
  RecordProperty("observer_summary_bytes",std::to_string(full.assembly.observer.bytes));
  EXPECT_TRUE(exact.InitializeMapped(21301,372435,full.bytes));
  EXPECT_FALSE(short_cap.InitializeMapped(21301,372435,full.bytes-1));
  ASSERT_TRUE(legacy.Initialize(21301,372435,std::size_t{2}<<30));
  EXPECT_EQ(legacy.assembly.observer.bytes,0u);
  EXPECT_EQ(m::ObserverBlocks(1,1),1u);EXPECT_EQ(m::ObserverBlocks(128,1),1u);
  EXPECT_EQ(m::ObserverBlocks(129,1),2u);EXPECT_EQ(m::ObserverBlocks(32769,1),256u);
  EXPECT_EQ(m::ObserverBlocks(std::size_t{UINT32_MAX},1),0u);
}
TEST(T3ObserverReduction,OriginalReadsetAndRemovedAcceptedCacheWork) {
  Fixture fixture;
  // This node is outside every source triangle. Joined Measure never required
  // its velocity or omega finite; only its position and quaternion are read.
  fixture.source.fields.velocity[3*(Nodes-1)]=NAN;
  fixture.source.fields.omega[3*(Nodes-1)]=INFINITY;
  auto& accepted=fixture.host->slab[0].element[0];
  Remove(fixture.host->model.element[0].reference,accepted,true);
  const auto actual=fixture.Staged();
  ASSERT_EQ(actual.status,q::BatchStatus::Success);
  SameExceptSums(actual,fixture.Serial());
  CheckTruth(fixture,actual);
  // Nonpositive native dt is a serial fallback domain, not new rejection.
  fixture.host->slab[1].element[0].diagnostics.unscaled_element_dt=-1;
  SameControl(fixture.Staged(),fixture.Serial());
  EXPECT_EQ(fixture.Staged().status,q::BatchStatus::Success);
  // A consumed nonfinite velocity still fails, with the original partial work.
  fixture.source.fields.velocity[0]=NAN;
  SameControl(fixture.Staged(),fixture.Serial());
  EXPECT_NE(fixture.Staged().status,q::BatchStatus::Success);
}
} // namespace t3_observer_test
