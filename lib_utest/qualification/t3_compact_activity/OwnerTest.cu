// SPDX-License-Identifier: MIT
#include "OwnerProbe.h"
#include "Flow.h"
namespace t3_compact_owner {
namespace old=t3_readback_test;namespace fe=tl::fea;namespace t=fe::t3;
TEST(T3CompactActivityOwner,ActualAcceptedAndPreparedQueriesMatchFullHistoriesAcrossRetryAndCommit) {
  old::Rig rig;ASSERT_TRUE(rig.Initialize());
  for(unsigned step=0;step<6;++step) {
    old::Oracle before;ASSERT_NO_FATAL_FAILURE(old::ReadOracle(rig,before));
    std::uint8_t flag=19;t::BatchDiagnostics diagnostics;Watch();
    ASSERT_EQ(rig.t3.CopyAcceptedParentActivity(rig.owner.accepted(),&flag,1,&diagnostics).status,t::BatchStatus::Success);
    EXPECT_EQ(old::transfers.copies,5u);EXPECT_EQ(old::transfers.bytes,25u);EXPECT_EQ(old::transfers.force_copies,0u);EXPECT_EQ(old::transfers.syncs,5u);
    Stop();EXPECT_EQ(flag,before.histories.sections[0].one_point()->point.failure.history.point_active);
    EXPECT_EQ(old::serial::Activity(before,0,diagnostics.time,diagnostics.epoch).status,t::BatchStatus::Success);
    fe::NodalTrialToken token;fe::NodalPreparedView view;fe::ShellPhysicalDiagnostics candidate;
    ASSERT_TRUE(rig.Prepare(token,view,candidate));old::Oracle after;ASSERT_NO_FATAL_FAILURE(old::ReadOracle(rig,after,&candidate.t3));
    Watch();ASSERT_EQ(rig.t3.CopyPreparedParentActivity(rig.owner,token,candidate.t3,&flag,1).status,t::BatchStatus::Success);
    EXPECT_EQ(old::transfers.bytes,25u);EXPECT_EQ(old::transfers.force_copies,0u);Stop();
    EXPECT_EQ(flag,after.histories.sections[0].one_point()->point.failure.history.point_active);
    if(!step){old::Discard(rig);ASSERT_TRUE(rig.Prepare(token,view,candidate));}
    ASSERT_NO_FATAL_FAILURE(old::Commit(rig,token,view,candidate));
  }
}
TEST(T3CompactActivityOwner,EveryCopyDrainAndPendingOrLaunchErrorPoisonsWithoutPublishing) {
  for(unsigned family=0;family<3;++family)for(unsigned point=1;point<=(family==2?8u:5u);++point) {
    SCOPED_TRACE(family);
    SCOPED_TRACE(point);Stop();old::Rig rig;ASSERT_TRUE(rig.Initialize());
    std::uint8_t flag=19;t::BatchDiagnostics diagnostics;const auto before=old::Bytes(diagnostics);Watch();
    if(family==0)probe.fail_copy=point;if(family==1)probe.fail_sync=point;if(family==2)probe.fail_error=point;
    EXPECT_EQ(rig.t3.CopyAcceptedParentActivity(rig.owner.accepted(),&flag,1,&diagnostics).status,t::BatchStatus::DeviceFailure);
    EXPECT_EQ(flag,19);EXPECT_EQ(old::Bytes(diagnostics),before);
    Watch();EXPECT_EQ(rig.t3.CopyAcceptedParentActivity(rig.owner.accepted(),&flag,1,&diagnostics).status,t::BatchStatus::DeviceFailure);
    EXPECT_EQ(old::transfers.copies,0u);Stop();
  }
}
TEST(T3CompactActivityOwner,MalformedPacketsFailClosedAndEveryRetryIsFresh) {
  for(unsigned phase=1;phase<=5;++phase) {
    Stop();old::Rig rig;ASSERT_TRUE(rig.Initialize());std::uint8_t flag=19;t::BatchDiagnostics diagnostics;
    Watch();probe.packet_corruption=phase;
    EXPECT_NE(rig.t3.CopyAcceptedParentActivity(rig.owner.accepted(),&flag,1,&diagnostics).status,t::BatchStatus::Success);EXPECT_EQ(flag,19);
    Watch();EXPECT_EQ(rig.t3.CopyAcceptedParentActivity(rig.owner.accepted(),&flag,1,&diagnostics).status,t::BatchStatus::Success);EXPECT_EQ(flag,1);Stop();
  }
}
TEST(T3CompactActivityOwner,PrivatePacketAliasesAndForeignOrStaleQueriesFailBeforeTransport) {
  old::Rig rig;ASSERT_TRUE(rig.Initialize());std::uint8_t flag=19;t::BatchDiagnostics diagnostic;
  Watch();ASSERT_EQ(rig.t3.CopyAcceptedParentActivity(rig.owner.accepted(),&flag,1,&diagnostic).status,t::BatchStatus::Success);
  auto* alias=static_cast<std::uint8_t*>(probe.packet_host);ASSERT_NE(alias,nullptr);const auto first=*alias;
  const auto stamp=rig.owner.accepted();auto stale=stamp;stale.epoch++;
  Watch();EXPECT_EQ(rig.t3.CopyAcceptedParentActivity(stamp,alias,1,&diagnostic).status,t::BatchStatus::InvalidInput);
  EXPECT_EQ(rig.t3.CopyAcceptedParentActivity(stale,&flag,1,&diagnostic).status,t::BatchStatus::StaleTrial);
  EXPECT_EQ(rig.t3.CopyAcceptedParentActivity(stamp,nullptr,1,&diagnostic).status,t::BatchStatus::InvalidInput);
  EXPECT_EQ(rig.t3.CopyAcceptedParentActivity(stamp,&flag,SIZE_MAX,&diagnostic).status,t::BatchStatus::ResourceLimit);
  EXPECT_EQ(old::transfers.copies,0u);EXPECT_EQ(*alias,first);Stop();
  fe::NodalTrialToken token;fe::NodalPreparedView view;fe::ShellPhysicalDiagnostics candidate;ASSERT_TRUE(rig.Prepare(token,view,candidate));
  Watch();fe::FENodalState foreign;EXPECT_NE(rig.t3.CopyPreparedParentActivity(foreign,token,candidate.t3,&flag,1).status,t::BatchStatus::Success);
  EXPECT_EQ(rig.t3.CopyPreparedParentActivity(rig.owner,token,candidate.t3,alias,1).status,t::BatchStatus::InvalidInput);
  EXPECT_EQ(old::transfers.copies,0u);Stop();old::Discard(rig);
}
TEST(T3CompactActivityOwner,PreparedNumericalFailureInvalidatesCandidateAndPreservesAcceptedRetry) {
  old::Rig rig;ASSERT_TRUE(rig.Initialize());old::Oracle accepted;ASSERT_NO_FATAL_FAILURE(old::ReadOracle(rig,accepted));
  fe::NodalTrialToken token;fe::NodalPreparedView view;fe::ShellPhysicalDiagnostics candidate;
  ASSERT_TRUE(rig.Prepare(token,view,candidate));const auto stamp=rig.owner.accepted();std::uint8_t flag=19;
  Watch();probe.packet_corruption=1;
  EXPECT_EQ(rig.t3.CopyPreparedParentActivity(rig.owner,token,candidate.t3,&flag,1).status,t::BatchStatus::NonfiniteResult);
  EXPECT_EQ(flag,19);Watch();
  EXPECT_EQ(rig.t3.CopyPreparedParentActivity(rig.owner,token,candidate.t3,&flag,1).status,t::BatchStatus::StaleTrial);
  EXPECT_EQ(old::transfers.copies,0u);Stop();old::Discard(rig);
  EXPECT_TRUE(fe::trial_identity::SameStamp(stamp,rig.owner.accepted()));
  old::Oracle after;ASSERT_NO_FATAL_FAILURE(old::ReadOracle(rig,after));
  EXPECT_EQ(accepted.staging[0].proposed_history.stamp().time,after.staging[0].proposed_history.stamp().time);
  EXPECT_EQ(accepted.staging[0].proposed_history.data().active,after.staging[0].proposed_history.data().active);
  ASSERT_TRUE(rig.Prepare(token,view,candidate));Watch();
  EXPECT_EQ(rig.t3.CopyPreparedParentActivity(rig.owner,token,candidate.t3,&flag,1).status,t::BatchStatus::Success);Stop();
  ASSERT_NO_FATAL_FAILURE(old::Commit(rig,token,view,candidate));
}
TEST(T3CompactActivityOwner,CompleteForceAndLayeredHistoryApisRemainFresh) {
  old::Rig rig;ASSERT_TRUE(rig.Initialize());t::ForceTrial result;t::BatchDiagnostics diagnostics;fe::ShellBatchLayeredSection section;
  Watch();ASSERT_EQ(rig.t3.CopyAcceptedResults(rig.owner.accepted(),&result,1,&diagnostics).status,t::BatchStatus::Success);
  EXPECT_EQ(old::transfers.force_copies,1u);EXPECT_EQ(old::transfers.copies,1u);
  Watch();ASSERT_EQ(rig.t3.CopyAcceptedLayeredSectionHistory(rig.owner.accepted(),&section,1,&diagnostics).status,t::BatchStatus::Success);
  EXPECT_EQ(old::transfers.force_copies,2u);EXPECT_EQ(old::transfers.copies,6u);EXPECT_EQ(old::transfers.syncs,5u);Stop();
}
}

namespace t3_compact_owner {
TEST(T3CompactActivityOwner,OwningForecastIncludesPacketsAtExactAndOneShortCaps) {
  old::Rig rig;ASSERT_TRUE(rig.Initialize());t::T3BatchConfig config;
  config.startup=rig.fixture.startup;config.owner=rig.owner.accepted();
  config.configuration_id=physical_publication_test::Configuration;config.qualification_id=physical_publication_test::Qualification;
  config.element_count=rig.fixture.physical.shells()->t3_count();config.usage=t::BatchUsage::CoupledForces;
  const auto original=t::T3Batch::ForecastMapped(config,rig.fixture.physical,rig.fixture.WitnessSource());
  ASSERT_EQ(original.report.status,t::BatchStatus::Success);
  config.max_device_bytes=original.footprint.device_bytes;
  config.storage_limits.max_host_bytes=original.footprint.startup_host_bytes;
  EXPECT_EQ(t::T3Batch::ForecastMapped(config,rig.fixture.physical,rig.fixture.WitnessSource()).report.status,t::BatchStatus::Success);
  auto short_device=config;--short_device.max_device_bytes;
  EXPECT_EQ(t::T3Batch::ForecastMapped(short_device,rig.fixture.physical,rig.fixture.WitnessSource()).report.status,t::BatchStatus::ResourceLimit);
  auto short_host=config;--short_host.storage_limits.max_host_bytes;
  EXPECT_EQ(t::T3Batch::ForecastMapped(short_host,rig.fixture.physical,rig.fixture.WitnessSource()).report.status,t::BatchStatus::ResourceLimit);
}
}

namespace t3_compact_owner {
TEST(T3CompactActivityOwner,ActualDeviceHistoryCorruptionKeepsFrozenPhasePriorityAndRetry) {
  for(bool prepared:{false,true})for(unsigned fault=0;fault<5;++fault) {
    Stop();old::Rig rig;ASSERT_TRUE(rig.Initialize());
    fe::NodalTrialToken token;fe::NodalPreparedView view;fe::ShellPhysicalDiagnostics candidate;
    if(prepared)ASSERT_TRUE(rig.Prepare(token,view,candidate));
    old::Oracle observed;probe={};probe.capture_sources=true;
    ASSERT_NO_FATAL_FAILURE(old::ReadOracle(rig,observed,prepared?&candidate.t3:nullptr));probe.capture_sources=false;
    const auto* force_device=probe.force_device;const auto* point_device=probe.point_device;
    ASSERT_NE(force_device,nullptr);ASSERT_NE(point_device,nullptr);
    t::ForceTrial original_force;fe::ShellBatchOnePointSectionState original_point;
    ASSERT_EQ(cudaMemcpy(&original_force,force_device,sizeof(original_force),cudaMemcpyDeviceToHost),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(&original_point,point_device,sizeof(original_point),cudaMemcpyDeviceToHost),cudaSuccess);
    auto force=original_force;auto point=original_point;
    if(fault==0||fault==2)point.point.reported_thickness_m*=2;
    if(fault==1||fault==2)force.diagnostics.native_sound_speed=std::numeric_limits<double>::quiet_NaN();
    if(fault==3)t3_compact_test::RawBool(point.point.failure.history.point_active,2);
    if(fault==4){const auto history=force.proposed_history.data();const auto stamp=force.proposed_history.stamp();
      ASSERT_EQ(t::PrepareFailurePrescribedHistory(rig.fixture.physical.shells()->t3_reference(0),history,
          {stamp.time+1,stamp.sample_index+1},force.proposed_history),t::Status::kSuccess);}
    t3_compact_test::Fixture oracle(1,0);
    oracle.elements[0].reference=rig.fixture.physical.shells()->t3_reference(0);oracle.forces[0]=force;oracle.points[0]=point;
    ASSERT_TRUE(rig.fixture.physical.catalog()->Parameters(fe::ShellBindingFamily::T3,0,&oracle.parameters[0]));
    oracle.time=oracle.force_time=prepared?candidate.t3.time:rig.owner.accepted().time;
    oracle.epoch=oracle.force_epoch=prepared?candidate.t3.epoch:rig.owner.accepted().epoch;
    const auto expected=t3_compact_test::Serial(oracle);ASSERT_NE(expected.status,t::BatchStatus::Success);
    ASSERT_EQ(cudaMemcpy(const_cast<void*>(force_device),&force,sizeof(force),cudaMemcpyHostToDevice),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(const_cast<void*>(point_device),&point,sizeof(point),cudaMemcpyHostToDevice),cudaSuccess);
    std::uint8_t flag=19;t::BatchDiagnostics diagnostic;
    const auto actual=prepared?rig.t3.CopyPreparedParentActivity(rig.owner,token,candidate.t3,&flag,1):
        rig.t3.CopyAcceptedParentActivity(rig.owner.accepted(),&flag,1,&diagnostic);
    t3_compact_test::Same(actual,expected);EXPECT_EQ(flag,19);
    ASSERT_EQ(cudaMemcpy(const_cast<void*>(force_device),&original_force,sizeof(original_force),cudaMemcpyHostToDevice),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(const_cast<void*>(point_device),&original_point,sizeof(original_point),cudaMemcpyHostToDevice),cudaSuccess);
    if(prepared){old::Discard(rig);ASSERT_TRUE(rig.Prepare(token,view,candidate));
      EXPECT_EQ(rig.t3.CopyPreparedParentActivity(rig.owner,token,candidate.t3,&flag,1).status,t::BatchStatus::Success);old::Discard(rig);}
    EXPECT_EQ(rig.t3.CopyAcceptedParentActivity(rig.owner.accepted(),&flag,1,&diagnostic).status,t::BatchStatus::Success);
  }
}
}
