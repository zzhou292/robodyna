// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Assertions.h"
#include "NativeOracle.h"
#include <limits>
namespace type25_lifecycle_test {
TEST(Type25Lifecycle, NativeColdRetainedSlidingDuplicatesAndSideRewriteCompose) {
  for(unsigned scenario=0;scenario<5;++scenario) {
    Fixture f;
    if(scenario==1){f.Retained();f.positions[18]=5;f.velocities[18]=3000;}
    if(scenario==2){f.spatial.push_back({1,1});f.Rebuild();}
    if(scenario==3)f.positions[20]=-.2;
    if(scenario==4){f.Retained();f.positions[20]=2;}
    SCOPED_TRACE(scenario);const auto actual=RunLifecycleFixture(f);Same(actual,OracleLifecycle(f.Input()));
    if(scenario==1) {
      ASSERT_EQ(actual.rows.size(),1u);EXPECT_EQ(actual.rows[0].optimized_count,0u);
      EXPECT_EQ(actual.rows[0].sliding_count,1u);EXPECT_EQ(actual.rows[0].continuation_count,1u);
      EXPECT_EQ(actual.rows[0].history.row.irtlm[0],33);
    }
    if(scenario==2)EXPECT_EQ(actual.rows[0].new_impact_count,2u);
    if(scenario==3) {
      ASSERT_EQ(actual.occurrences.size(),1u);EXPECT_EQ(actual.occurrences[0].local_main,2);
      EXPECT_EQ(actual.occurrences[0].selected.key.main_segment,22);
    }
  }
}
TEST(Type25Lifecycle, RuntimeContactFlagPersistsAcrossLossReacquisitionAndRetry) {
  Fixture f;f.Retained();f.secondary[0].initial_contact_flag=-1;
  f.positions[20]=2;f.velocities[20]=1800;
  const auto lost=RunLifecycleFixture(f);Same(lost,OracleLifecycle(f.Input()));
  ASSERT_EQ(lost.rows[0].initial_contact_flag,0);ASSERT_EQ(lost.rows[0].history.row.irtlm[0],0);
  auto finished=OracleFinish(lost.rows);
  f.accepted[0]=finished[0].history;f.secondary[0].initial_contact_flag=finished[0].initial_contact_flag;
  f.positions[20]=.2;f.velocities[20]=-1800;f.step.time=.001;
  l::HostResult prior=lost;auto output=prior;auto limits=Fixture::Limits();limits.candidates=0;
  const auto stopped=l::EvaluateNativeLifecycleHost(f.Input(),limits,&output);
  EXPECT_EQ(stopped.status,n::selection::Status::CapacityExceeded);EXPECT_TRUE(stopped.count_complete);
  Same(output,prior,true);EXPECT_EQ(f.secondary[0].initial_contact_flag,0);
  const auto reacquired=RunLifecycleFixture(f);Same(reacquired,OracleLifecycle(f.Input()));
  ASSERT_GT(reacquired.rows[0].kept_count,0u);
  auto incorrectly_reseeded=f;incorrectly_reseeded.secondary[0].initial_contact_flag=-1;
  const auto wrong=RunLifecycleFixture(incorrectly_reseeded);
  EXPECT_EQ(wrong.rows[0].kept_count,0u);
}
TEST(Type25Lifecycle, OptcdPrecisionVelocityScreenAndDeletedMainDelayAreNative) {
  for(int precision:{0,1,2}) {
    Fixture f;f.profile.optcd_response_precision=precision;
    f.secondary[0].gap=0;for(auto& m:f.mains){m.maximum_gap=0;for(auto& gap:m.gap)gap=0;}
    f.positions[20]=1e-6;
    const auto out=RunLifecycleFixture(f);Same(out,OracleLifecycle(f.Input()));
    EXPECT_EQ(out.rows[0].optimized_count,precision==1?1u:0u);
  }
  Fixture f;f.Retained();f.mains[0].coefficient=0;f.positions[18]=5;
  auto out=RunLifecycleFixture(f);Same(out,OracleLifecycle(f.Input()));
  ASSERT_TRUE(out.occurrences.empty());EXPECT_EQ(out.rows[0].history.row.irtlm[2],0);
  f.accepted[0]=OracleFinish(out.rows)[0].history;
  f.secondary[0].initial_contact_flag=out.rows[0].initial_contact_flag;
  out=RunLifecycleFixture(f);Same(out,OracleLifecycle(f.Input()));
  EXPECT_GT(out.rows[0].kept_count,0u);
}
TEST(Type25Lifecycle, SourceNeighborOwnNodeAndSymmetricExclusionsPreserveAppendOrder) {
  for(unsigned exclusion=0;exclusion<3;++exclusion) {
    Fixture f;f.Retained();f.positions[18]=5;
    if(exclusion==0){f.removed_entries={3};f.removed_offsets={0,1};}
    if(exclusion==1)f.mains[2].nodes[0]=f.secondary[0].node;
    if(exclusion==2)f.mains[2].coefficient=0;
    const auto out=RunLifecycleFixture(f);Same(out,OracleLifecycle(f.Input()));
    EXPECT_EQ(out.rows[0].sliding_count,0u);EXPECT_EQ(out.rows[0].kept_count,0u);
  }
}
TEST(Type25Lifecycle, NativeAndSiBorrowedKinematicsHaveCompleteReferenceParity) {
  Fixture f;f.Retained();f.positions[18]=5;f.velocities[18]=3000;
  const auto native=RunLifecycleFixture(f);
  f.units=l::KinematicsUnits::Si;
  for(auto& value:f.positions)value*=f.scale.length_m;
  for(auto& value:f.velocities)value*=f.scale.length_m/f.scale.time_s;
  const auto si=RunLifecycleFixture(f);Same(si,OracleLifecycle(f.Input()));Same(si,native);
}
TEST(Type25Lifecycle, BadCsrGenerationAndCapLeaveOutputUntouchedBeforeValidRetry) {
  Fixture f;const auto before=RunLifecycleFixture(f);
  for(unsigned fault=0;fault<4;++fault) {
    auto broken=f;auto output=before;auto limits=Fixture::Limits();
    if(fault==0)broken.spatial_entries[1]=0;
    if(fault==1)broken.accepted[0].generation++;
    if(fault==2)limits.scratch_bytes=0;
    if(fault==3)broken.profile.optcd_response_precision=-1;
    const auto report=l::EvaluateNativeLifecycleHost(broken.Input(),limits,&output);
    EXPECT_NE(report.status,n::selection::Status::Ok);Same(output,before,true);
  }
  Same(RunLifecycleFixture(f),before,true);
}
TEST(Type25Lifecycle, PostForceNormalizationIsExplicitAndPreservesEveryOtherField) {
  Fixture f;const auto selected=RunLifecycleFixture(f);const auto finished=OracleFinish(selected.rows);
  ASSERT_LT(selected.rows[0].history.row.irtlm[0],0);
  n::NativeGeometryHistory actual;
  ASSERT_EQ(l::FinishNativeRow(selected.rows[0].history,&actual),n::selection::Status::Ok);
  type25_geometry_test::Same(actual,finished[0].history,true);
  EXPECT_LT(selected.rows[0].history.row.irtlm[0],0);
}

TEST(Type25Lifecycle, InactiveUnderflowAndUnboundNativeScratchAreNotInventedContacts) {
  for(unsigned scenario=0;scenario<3;++scenario) {
    Fixture f;
    if(scenario==0)f.secondary[0].coefficient=0;
    if(scenario==1){f.secondary[0].coefficient=0x1p-1022;for(auto& m:f.mains)m.coefficient=0x1p-1022;}
    if(scenario==2)for(auto& normal:f.normals)for(auto& v:normal.bisector)v.x=std::numeric_limits<float>::quiet_NaN();
    const auto actual=RunLifecycleFixture(f);Same(actual,OracleLifecycle(f.Input()));
    if(scenario<2)EXPECT_EQ(actual.rows[0].kept_count,0u);
    if(scenario==1) {
      ASSERT_GT(actual.occurrences.size(),0u);EXPECT_EQ(actual.rows[0].new_impact_count,actual.occurrences.size());
      for(const auto& occurrence:actual.occurrences)EXPECT_TRUE(occurrence.cache_initialized);
    }
  }
}
TEST(Type25Lifecycle, EmptySecondaryRosterAndExactCountCapacityRemainBounded) {
  Fixture f;auto before=RunLifecycleFixture(f);auto out=before;
  const auto count=before.occurrences.size();ASSERT_GT(count,0u);
  auto limit=Fixture::Limits();limit.candidates=count-1;
  auto report=l::EvaluateNativeLifecycleHost(f.Input(),limit,&out);
  EXPECT_EQ(report.status,n::selection::Status::CapacityExceeded);
  EXPECT_TRUE(report.count_complete);EXPECT_EQ(report.required_candidates,count);Same(out,before,true);
  limit.candidates=count;
  ASSERT_EQ(l::EvaluateNativeLifecycleHost(f.Input(),limit,&out).status,n::selection::Status::Ok);Same(out,before,true);
  f.secondary.clear();f.accepted.clear();f.spatial.clear();f.Rebuild();
  report=l::EvaluateNativeLifecycleHost(f.Input(),Fixture::Limits(),&out);
  EXPECT_EQ(report.status,n::selection::Status::Ok);EXPECT_TRUE(report.count_complete);
  EXPECT_EQ(report.required_candidates,0u);EXPECT_TRUE(out.rows.empty());EXPECT_TRUE(out.occurrences.empty());
}

TEST(Type25Lifecycle, MixedTriangleQuadUsesNativeThreeSlotSlidingAndAdjacency) {
  for(bool retained:{false,true}) {
    Fixture f;f.TrianglePair();ASSERT_EQ(f.normal_entries.size(),10u);
    if(retained){f.Retained();f.positions[18]=5;f.positions[19]=1;}
    const auto actual=RunLifecycleFixture(f);Same(actual,OracleLifecycle(f.Input()));
    EXPECT_EQ(actual.rows[0].sliding_reference[3],0);
    if(retained) {
      EXPECT_GT(actual.rows[0].sliding_count,0u);EXPECT_GT(actual.rows[0].continuation_count,0u);
    }
  }
}
TEST(Type25Lifecycle, LateRowFailureRetainsCompleteGlobalRequiredCountAndPriorOutput) {
  Fixture f;auto out=RunLifecycleFixture(f),before=out;
  f.AddSecondary(5,1,.2);f.spatial={{1,1},{2,3}};f.Rebuild();
  f.mains[2].coefficient=std::numeric_limits<double>::max();
  const auto report=l::EvaluateNativeLifecycleHost(f.Input(),Fixture::Limits(),&out);
  EXPECT_EQ(report.status,n::selection::Status::NonfiniteResult);EXPECT_EQ(report.stage,l::Stage::NewImpact);
  EXPECT_EQ(report.secondary,1u);EXPECT_TRUE(report.count_complete);EXPECT_EQ(report.required_candidates,2u);
  Same(out,before,true);
  f.mains[2].coefficient=400;Same(RunLifecycleFixture(f),OracleLifecycle(f.Input()));
}
TEST(Type25Lifecycle, ExplicitSignedInactiveMainsMatchNativeWithoutChangingLegacyAdmission) {
  for(bool retained:{false,true}) {
    Fixture f;const auto before=RunLifecycleFixture(f);auto out=before;
    if(retained){f.Retained();f.positions[18]=5;f.mains[2].coefficient=-400;}
    else f.mains[0].coefficient=-400;
    EXPECT_EQ(l::EvaluateNativeLifecycleHost(f.Input(),Fixture::Limits(),&out).status,n::selection::Status::InvalidInput);
    Same(out,before,true);
    f.profile.main_coefficient_domain=n::MainCoefficientDomain::NativeSigned;
    const auto actual=RunLifecycleFixture(f);Same(actual,OracleLifecycle(f.Input()));
    if(!retained)EXPECT_EQ(actual.rows[0].optimized_count,0u);
    else EXPECT_EQ(actual.rows[0].sliding_count,0u);
    f.profile.main_coefficient_domain=static_cast<n::MainCoefficientDomain>(91);
    EXPECT_EQ(l::EvaluateNativeLifecycleHost(f.Input(),Fixture::Limits(),&out).status,n::selection::Status::UnsupportedProfile);
    Same(out,before,true);
  }
}
} // namespace type25_lifecycle_test
