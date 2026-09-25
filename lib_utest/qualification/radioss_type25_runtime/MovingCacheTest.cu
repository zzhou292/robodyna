// SPDX-License-Identifier: AGPL-3.0-or-later
#include "MovingCacheRig.h"
#include "../radioss_type25_current_normals/NativeOracle.h"
#include "lib_src/collision/radioss_type25/normal_activation/Values.h"
#include <gtest/gtest.h>
#include <cstring>
namespace moving_cache_test {
namespace q=n::runtime_qualification;namespace c=n::current_normals;
void Same(const q::NormalObservation& a,const q::NormalObservation& b) {
  ASSERT_EQ(a.face.size(),b.face.size());ASSERT_EQ(a.references.size(),b.references.size());
  EXPECT_EQ(std::memcmp(a.face.data(),b.face.data(),a.face.size()*sizeof(n::StoredNormal)),0);
  EXPECT_EQ(std::memcmp(a.references.data(),b.references.data(),a.references.size()*sizeof(n::startup::NormalReference)),0);
}
void NativeExpected(Rig& rig,const std::vector<double>& positions,const q::NormalObservation& prior,
    const q::NormalObservation& actual) {
  const auto source=rig.source.Source();std::vector<double> coefficients;std::vector<std::uint32_t> free;
  for(std::size_t i=0;i<source.selection.main_count;++i) {
    coefficients.push_back(source.selection.mains[i].coefficient);
    if(n::normal_activation::detail::FreeMain(source.selection.mains[i]))free.push_back(std::uint32_t(i+1));
  }
  // Every face in this two disconnected shell fixture has a native free edge;
  // FREE_BOUND/TAGN must activate the complete main and physical node roster.
  ASSERT_EQ(free,(std::vector<std::uint32_t>{1,2,3,4}));
  EXPECT_EQ(actual.active,(std::vector<std::uint32_t>(4,1)));
  EXPECT_EQ(actual.tags,(std::vector<std::uint32_t>(7,1)));
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
} // namespace moving_cache_test
