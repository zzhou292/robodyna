// SPDX-License-Identifier: AGPL-3.0-or-later
#include "MovingCacheRig.h"
#include "../radioss_type25_current_normals/NativeOracle.h"
#include "lib_src/collision/radioss_type25/normal_activation/Values.h"
#include <gtest/gtest.h>
#include <cstring>
namespace moving_cache_test {
namespace q=n::runtime_qualification;namespace c=n::current_normals;
void SameCandidateReport(const n::candidates::Report& a,const n::candidates::Report& b) {
  EXPECT_EQ(a.status,b.status);EXPECT_EQ(a.failure_row,b.failure_row);
  EXPECT_EQ(a.stamp.source.source,b.stamp.source.source);EXPECT_EQ(a.stamp.source.topology,b.stamp.source.topology);
  EXPECT_EQ(a.stamp.activity,b.stamp.activity);EXPECT_EQ(a.stamp.gaps,b.stamp.gaps);
  EXPECT_EQ(a.stamp.geometry,b.stamp.geometry);EXPECT_EQ(a.stamp.attempt,b.stamp.attempt);EXPECT_EQ(a.stamp.reference,b.stamp.reference);
  EXPECT_EQ(a.active_secondaries,b.active_secondaries);EXPECT_EQ(a.envelope_encounters,b.envelope_encounters);
  EXPECT_EQ(a.tasks,b.tasks);EXPECT_EQ(a.pairs,b.pairs);EXPECT_EQ(a.maximum_secondary_gap,b.maximum_secondary_gap);
  EXPECT_EQ(a.own_kernel_launches,b.own_kernel_launches);EXPECT_EQ(a.sort_calls,b.sort_calls);
  EXPECT_EQ(a.scan_calls,b.scan_calls);EXPECT_EQ(a.host_fences,b.host_fences);
  EXPECT_EQ(a.strategy,b.strategy);EXPECT_EQ(a.encounters_counted,b.encounters_counted);EXPECT_EQ(a.pairs_counted,b.pairs_counted);
}
void Same(const q::NormalObservation& a,const q::NormalObservation& b) {
  ASSERT_EQ(a.face.size(),b.face.size());ASSERT_EQ(a.references.size(),b.references.size());
  EXPECT_EQ(std::memcmp(a.face.data(),b.face.data(),a.face.size()*sizeof(n::StoredNormal)),0);
  EXPECT_EQ(std::memcmp(a.references.data(),b.references.data(),a.references.size()*sizeof(n::startup::NormalReference)),0);
}
void NativeExpected(Rig& rig,const std::vector<double>& positions,const q::NormalObservation& prior,
    const q::NormalObservation& actual,bool inactive_triangle=false) {
  const auto source=rig.source.Source();std::vector<double> coefficients;std::vector<std::uint32_t> free;
  for(std::size_t i=0;i<source.selection.main_count;++i) {
    coefficients.push_back(source.selection.mains[i].coefficient);
    if(n::normal_activation::detail::FreeMain(source.selection.mains[i]))free.push_back(std::uint32_t(i+1));
  }
  // Exact independently known masks for these disconnected free-edge shells.
  // Constant-zero triangle coefficients omit both triangle sides and nodes.
  ASSERT_EQ(free,(inactive_triangle?std::vector<std::uint32_t>{1,3}:std::vector<std::uint32_t>{1,2,3,4}));
  EXPECT_EQ(actual.active,(inactive_triangle?std::vector<std::uint32_t>{1,0,1,0}:std::vector<std::uint32_t>(4,1)));
  EXPECT_EQ(actual.tags,(inactive_triangle?std::vector<std::uint32_t>{1,1,1,1,0,0,0}:std::vector<std::uint32_t>(7,1)));
  c::Input in;in.profile=c::Profile::OrdinaryShellLocal;in.free_roster=n::normal_activation::FreeRosterPolicy::FreshComplete;
  in.topology={source.starter.mains,source.selection.node_count,source.primary_main_count,source.selection.main_count,
      source.selection.normal_count,source.selection.normal_to_main};
  in.positions={positions.data(),std::uint32_t(source.selection.node_count),3,1};in.coordinates=n::startup::Coordinates::Si;
  in.units=rig.config.units;in.main_coefficients=coefficients.data();in.coefficient_count=coefficients.size();
  in.main_active=actual.active.data();in.active_count=actual.active.size();in.node_tag=actual.tags.data();in.tag_count=actual.tags.size();
  in.free_main_ids=free.data();in.free_count=free.size();in.prior_normals=prior.face.data();in.prior_count=prior.face.size();
  const auto expected=type25_current_normals_test::Oracle(in);ASSERT_TRUE(expected.finite);
  q::NormalObservation comparison;comparison.face=expected.normals;comparison.references=expected.references;Same(actual,comparison);
}
TEST(NativeMovingCacheCuda, RealOwnerCacheUpdatesMatchNativeAndPublishAtTheForceBaseEpoch) {
  Rig rig;rig.Initialize();q::NormalObservation accepted;
  ASSERT_TRUE(q::Access::ReadAcceptedNormals(rig.contact,&accepted));
  EXPECT_FALSE(accepted.accepted.force_phase_available);EXPECT_EQ(accepted.accepted.generation,0u);
  ASSERT_EQ(accepted.face.size(),4*rig.source.starter.main_count);
  EXPECT_EQ(std::memcmp(accepted.face.data(),rig.source.starter.starter.face_normals,accepted.face.size()*sizeof(n::StoredNormal)),0);
  const auto footprint=rig.contact.allocations();EXPECT_GT(footprint.normal_device_bytes,0u);
  for(unsigned step=0;step<8;++step) {
    SCOPED_TRACE(step);
    const auto x=rig.Positions();Attempt a;rig.Begin(a);q::NormalObservation staged;
    EXPECT_FALSE(q::Access::ReadAttemptNormals(rig.contact,rig.owner,a.token,a.assembly,&staged));
    Check(rig.contact.AssembleAccepted(rig.owner,a.token,a.assembly));
    ASSERT_TRUE(q::Access::ReadAttemptNormals(rig.contact,rig.owner,a.token,a.assembly,&staged));
    EXPECT_EQ(staged.force_base_epoch,step);EXPECT_EQ(staged.attempt,a.assembly.attempt);
    NativeExpected(rig,x,accepted,staged);
    q::NormalObservation unchanged;ASSERT_TRUE(q::Access::ReadAcceptedNormals(rig.contact,&unchanged));Same(unchanged,accepted);
    rig.Prepare(a);Check(rig.Commit(a));
    ASSERT_TRUE(q::Access::ReadAcceptedNormals(rig.contact,&accepted));Same(accepted,staged);
    EXPECT_TRUE(accepted.accepted.force_phase_available);EXPECT_EQ(accepted.accepted.generation,step+1);
    EXPECT_EQ(accepted.accepted.stamp.epoch,step+1);EXPECT_EQ(accepted.accepted.force_base_stamp.epoch,step);
    EXPECT_FALSE(q::Access::ReadAttemptNormals(rig.contact,rig.owner,a.token,a.assembly,&staged));
    EXPECT_EQ(rig.contact.allocations().device_bytes,footprint.device_bytes);
  }
}
TEST(NativeMovingCacheCuda, CommonRejectionAndDiscardRetryKeepAcceptedCacheAndHistorySelectors) {
  Rig rig;rig.Initialize();q::NormalObservation original,first,retry;
  ASSERT_TRUE(q::Access::ReadAcceptedNormals(rig.contact,&original));const auto x=rig.Positions();
  Attempt rejected;rig.Begin(rejected);Check(rig.contact.AssembleAccepted(rig.owner,rejected.token,rejected.assembly));
  ASSERT_TRUE(q::Access::ReadAttemptNormals(rig.contact,rig.owner,rejected.token,rejected.assembly,&first));
  rig.Prepare(rejected);EXPECT_NE(rig.Commit(rejected,false).status,fe::ShellPublicationStatus::Success);
  q::NormalObservation unchanged;ASSERT_TRUE(q::Access::ReadAcceptedNormals(rig.contact,&unchanged));Same(unchanged,original);
  EXPECT_EQ(unchanged.accepted.selectors.history,original.accepted.selectors.history);EXPECT_EQ(rig.Positions(),x);
  rig.Discard();EXPECT_FALSE(q::Access::ReadAttemptNormals(rig.contact,rig.owner,rejected.token,rejected.assembly,&retry));
  Attempt a;rig.Begin(a);Check(rig.contact.AssembleAccepted(rig.owner,a.token,a.assembly));
  ASSERT_TRUE(q::Access::ReadAttemptNormals(rig.contact,rig.owner,a.token,a.assembly,&retry));Same(retry,first);
  EXPECT_NE(a.assembly.attempt,rejected.assembly.attempt);rig.Prepare(a);Check(rig.Commit(a));
  q::NormalObservation accepted;ASSERT_TRUE(q::Access::ReadAcceptedNormals(rig.contact,&accepted));Same(accepted,retry);
  EXPECT_EQ(accepted.accepted.generation,1u);
}
TEST(NativeMovingCacheCuda, CompleteOptimizedCapacityRejectsBeforeNormalCacheCanPublish) {
  Rig rig;for(auto& row:rig.source.secondary)row.gap=.04;
  for(auto& main:rig.source.mains){main.maximum_gap=.04;for(auto& gap:main.gap)gap=.04;}
  auto limits=rig.Limits();limits.optimized_candidates=1;rig.Initialize(limits);
  q::NormalObservation original;ASSERT_TRUE(q::Access::ReadAcceptedNormals(rig.contact,&original));const auto x=rig.Positions();
  for(unsigned attempt=0;attempt<2;++attempt) {
    SCOPED_TRACE(attempt);
    Attempt a;rig.Begin(a);const auto failed=rig.contact.AssembleAccepted(rig.owner,a.token,a.assembly);
    EXPECT_EQ(failed.status,n::TransactionStatus::ResourceLimit)<<failed.message;
    q::NormalObservation unchanged;ASSERT_TRUE(q::Access::ReadAcceptedNormals(rig.contact,&unchanged));Same(unchanged,original);
    EXPECT_EQ(unchanged.accepted.generation,0u);EXPECT_EQ(rig.Positions(),x);
    EXPECT_FALSE(q::Access::ReadAttemptNormals(rig.contact,rig.owner,a.token,a.assembly,&unchanged));rig.Discard();
  }
}
TEST(NativeMovingCacheCuda, ExactCandidateTaskAndPairFailuresSurviveCommonDiscardAndRefreshOnRetry) {
  for(bool task_cap:{false,true}) {
    SCOPED_TRACE(task_cap);
    Rig rig;for(auto& row:rig.source.secondary)row.gap=.04;
    for(auto& main:rig.source.mains){main.maximum_gap=.04;for(auto& gap:main.gap)gap=.04;}
    auto limits=rig.Limits();
    if(task_cap)limits.inventory.max_tasks=1;else limits.inventory.max_pairs=1;
    ASSERT_NO_THROW(rig.Initialize(limits));
    EXPECT_FALSE(rig.contact.last_diagnostics().candidate_rebuild_available);
    const auto original_positions=rig.Positions();q::NormalObservation original;
    ASSERT_TRUE(q::Access::ReadAcceptedNormals(rig.contact,&original));
    std::uint64_t previous_attempt=0;
    for(unsigned retry=0;retry<2;++retry) {
      SCOPED_TRACE(retry);
      Attempt attempt;
      ASSERT_NO_THROW(rig.Begin(attempt));
      const auto failure=rig.contact.AssembleAccepted(rig.owner,attempt.token,attempt.assembly);
      ASSERT_EQ(failure.status,n::TransactionStatus::ResourceLimit)<<failure.message;
      const auto diagnostics=rig.contact.last_diagnostics();ASSERT_TRUE(diagnostics.candidate_rebuild_available);
      const auto& report=diagnostics.candidate_rebuild;EXPECT_EQ(report.status,n::candidates::Status::ResourceLimit);
      EXPECT_EQ(report.stamp.attempt,attempt.assembly.attempt);EXPECT_NE(report.stamp.attempt,previous_attempt);
      previous_attempt=report.stamp.attempt;EXPECT_GT(report.active_secondaries,0u);EXPECT_GT(report.envelope_encounters,0u);
      if(task_cap) {
        EXPECT_GT(report.tasks,limits.inventory.max_tasks);EXPECT_EQ(report.pairs,0u);
        EXPECT_EQ(report.host_fences,1u);EXPECT_EQ(report.scan_calls,1u);
      } else {
        EXPECT_LE(report.tasks,limits.inventory.max_tasks);EXPECT_GT(report.pairs,limits.inventory.max_pairs);
        EXPECT_EQ(report.host_fences,2u);EXPECT_EQ(report.scan_calls,2u);
      }
      n::candidates::Report actual;
      ASSERT_TRUE(q::Access::ReadCandidateReport(rig.contact,attempt.assembly.attempt,&actual));SameCandidateReport(report,actual);
      rig.Discard();const auto retained=rig.contact.last_diagnostics();ASSERT_TRUE(retained.candidate_rebuild_available);
      SameCandidateReport(retained.candidate_rebuild,report);
      EXPECT_EQ(rig.owner.accepted().epoch,0u);EXPECT_EQ(rig.Positions(),original_positions);
      q::NormalObservation unchanged;ASSERT_TRUE(q::Access::ReadAcceptedNormals(rig.contact,&unchanged));Same(unchanged,original);
      EXPECT_EQ(unchanged.accepted.generation,0u);EXPECT_EQ(diagnostics.active_forces,0u);
    }
  }
}
TEST(NativeMovingCacheCuda, CompactInventoryUsesPairedForecastAndRealOwnerCommitDiscardRetry) {
  Rig rig;for(auto& row:rig.source.secondary)row.gap=.04;
  for(auto& main:rig.source.mains){main.maximum_gap=.04;for(auto& gap:main.gap)gap=.04;}
  auto limits=rig.Limits();n::TransactionForecast legacy,compact;
  ASSERT_EQ(n::Transaction::Preflight(rig.config,rig.source.Source(),rig.source.physical.physical,legacy,limits).status,n::TransactionStatus::Ok);
  limits.inventory.strategy=n::candidates::EnumerationStrategy::CompactGrid;limits.inventory.max_encounters=128;
  ASSERT_EQ(n::Transaction::Preflight(rig.config,rig.source.Source(),rig.source.physical.physical,compact,limits).status,n::TransactionStatus::Ok);
  EXPECT_GT(compact.inventory_device_bytes,legacy.inventory_device_bytes);
  EXPECT_EQ(compact.device_bytes-legacy.device_bytes,compact.inventory_device_bytes-legacy.inventory_device_bytes);
  EXPECT_EQ(compact.runtime_device_bytes,legacy.runtime_device_bytes);
  limits.max_device_bytes=compact.device_bytes;limits.max_host_bytes=compact.startup_host_bytes;
  ASSERT_NO_THROW(rig.Initialize(limits));
  EXPECT_EQ(rig.contact.allocations().device_bytes,compact.device_bytes);
  q::NormalObservation original,first,retry;
  ASSERT_TRUE(q::Access::ReadAcceptedNormals(rig.contact,&original));const auto x=rig.Positions();
  Attempt rejected;
  ASSERT_NO_THROW(rig.Begin(rejected));
  ASSERT_EQ(rig.contact.AssembleAccepted(rig.owner,rejected.token,rejected.assembly).status,n::TransactionStatus::Ok);
  const auto diagnostics=rig.contact.last_diagnostics();ASSERT_TRUE(diagnostics.candidate_rebuild_available);
  EXPECT_EQ(diagnostics.candidate_rebuild.strategy,n::candidates::EnumerationStrategy::CompactGrid);
  EXPECT_TRUE(diagnostics.candidate_rebuild.encounters_counted);EXPECT_TRUE(diagnostics.candidate_rebuild.pairs_counted);
  EXPECT_GT(diagnostics.active_forces,0u);
  ASSERT_TRUE(q::Access::ReadAttemptNormals(rig.contact,rig.owner,rejected.token,rejected.assembly,&first));
  ASSERT_NO_THROW(rig.Prepare(rejected));
  EXPECT_NE(rig.Commit(rejected,false).status,fe::ShellPublicationStatus::Success);
  rig.Discard();q::NormalObservation unchanged;
  ASSERT_TRUE(q::Access::ReadAcceptedNormals(rig.contact,&unchanged));Same(unchanged,original);
  EXPECT_EQ(rig.owner.accepted().epoch,0u);EXPECT_EQ(rig.Positions(),x);
  Attempt attempt;
  ASSERT_NO_THROW(rig.Begin(attempt));
  ASSERT_EQ(rig.contact.AssembleAccepted(rig.owner,attempt.token,attempt.assembly).status,n::TransactionStatus::Ok);
  EXPECT_EQ(rig.contact.last_diagnostics().active_forces,diagnostics.active_forces);
  EXPECT_EQ(rig.contact.last_diagnostics().candidate_rebuild.pairs,diagnostics.candidate_rebuild.pairs);
  ASSERT_TRUE(q::Access::ReadAttemptNormals(rig.contact,rig.owner,attempt.token,attempt.assembly,&retry));Same(retry,first);
  ASSERT_NO_THROW(rig.Prepare(attempt));
  ASSERT_EQ(rig.Commit(attempt).status,fe::ShellPublicationStatus::Success);
  EXPECT_EQ(rig.owner.accepted().epoch,1u);EXPECT_TRUE(rig.contact.accepted().available);
}
TEST(NativeMovingCacheCuda, InactiveMainCacheSurvivesPhysicalMotionAndRepeatedSelectorSwaps) {
  Rig rig;rig.source.mains[1].coefficient=0;rig.source.mains[3].coefficient=0;rig.Initialize();
  q::NormalObservation prior;ASSERT_TRUE(q::Access::ReadAcceptedNormals(rig.contact,&prior));
  const auto initial=prior;const auto original_position=rig.Positions();
  for(unsigned step=0;step<8;++step) {
    SCOPED_TRACE(step);
    const auto x=rig.Positions();Attempt a;rig.Begin(a);Check(rig.contact.AssembleAccepted(rig.owner,a.token,a.assembly));
    q::NormalObservation staged;ASSERT_TRUE(q::Access::ReadAttemptNormals(rig.contact,rig.owner,a.token,a.assembly,&staged));
    NativeExpected(rig,x,prior,staged,true);
    for(std::size_t main:{1u,3u})
      EXPECT_EQ(std::memcmp(staged.face.data()+4*main,initial.face.data()+4*main,4*sizeof(n::StoredNormal)),0);
    rig.Prepare(a);Check(rig.Commit(a));ASSERT_TRUE(q::Access::ReadAcceptedNormals(rig.contact,&prior));Same(prior,staged);
    EXPECT_EQ(prior.accepted.force_base_stamp.epoch,step);
  }
  EXPECT_NE(rig.Positions(),original_position);
}
TEST(NativeMovingCacheCuda, GlobalLaw1AndGeneralStartupShareMovingContactAndCommonPublication) {
  Rig rig(true);rig.Initialize();
  EXPECT_EQ(rig.source.starter.topology,n::startup::TopologyPolicy::NativeOrdinaryShell);
  for(auto family:{fe::ShellBindingFamily::Qeph,fe::ShellBindingFamily::T3}) {
    fe::ShellSectionLaw law=fe::ShellSectionLaw::Unspecified;
    ASSERT_TRUE(rig.source.physical.catalog.Law(family,0,&law));EXPECT_EQ(law,fe::ShellSectionLaw::GlobalLaw1Npt0);
  }
  q::NormalObservation prior;ASSERT_TRUE(q::Access::ReadAcceptedNormals(rig.contact,&prior));
  std::size_t active_steps=0;
  for(unsigned step=0;step<4;++step) {
    SCOPED_TRACE(step);
    const auto x=rig.Positions();Attempt a;rig.Begin(a);Check(rig.contact.AssembleAccepted(rig.owner,a.token,a.assembly));
    active_steps+=rig.contact.last_diagnostics().active_forces!=0;
    q::NormalObservation staged;ASSERT_TRUE(q::Access::ReadAttemptNormals(rig.contact,rig.owner,a.token,a.assembly,&staged));
    NativeExpected(rig,x,prior,staged);rig.Prepare(a);Check(rig.Commit(a));
    ASSERT_TRUE(q::Access::ReadAcceptedNormals(rig.contact,&prior));Same(prior,staged);
    EXPECT_EQ(prior.accepted.force_base_stamp.epoch,step);
  }
  EXPECT_GT(active_steps,0u);EXPECT_EQ(rig.owner.accepted().epoch,4u);
}
} // namespace moving_cache_test
