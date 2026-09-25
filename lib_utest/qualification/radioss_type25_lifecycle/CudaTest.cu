// SPDX-License-Identifier: AGPL-3.0-or-later
#include "CudaFixture.h"
#include "NativeOracle.h"
namespace type25_lifecycle_test {
TEST(Type25LifecycleCuda, CompleteNativePhasesUseTheSameRowMathAtBothThreadShapes) {
  device::Device gpu;
  for(unsigned scenario=0;scenario<7;++scenario) {
    Fixture f;
    if(scenario==1){f.Retained();f.positions[18]=5;f.velocities[18]=3000;}
    if(scenario==2){f.spatial.push_back({1,1});f.Rebuild();}
    if(scenario==3)f.positions[20]=-.2;
    if(scenario==4){f.Retained();f.positions[20]=2;}
    if(scenario==5){f.secondary[0].coefficient=0x1p-1022;for(auto& m:f.mains)m.coefficient=0x1p-1022;}
    if(scenario==6){f.Retained();f.positions[18]=5;f.units=l::KinematicsUnits::Si;
      for(auto& x:f.positions)x*=f.scale.length_m;}
    SCOPED_TRACE(scenario);const auto cpu=RunLifecycleFixture(f);const auto native=OracleLifecycle(f.Input());
    for(unsigned threads:{1u,32u}) {
      l::HostResult actual;ASSERT_EQ(gpu.Evaluate(f,actual,threads==32,512,threads).status,n::selection::Status::Ok);
      Same(actual,cpu,true);Same(actual,native);
    }
  }
}
TEST(Type25LifecycleCuda, IndependentRowsReconstructNativeGlobalOccurrenceOrder) {
  Fixture f;f.Retained();f.positions[18]=5;
  f.nodes.push_back({107,0,0});f.positions.insert(f.positions.end(),{2,1,.2});
  f.velocities.insert(f.velocities.end(),{0,0,0});f.secondary.push_back({7,2,.2,0});
  f.accepted.push_back({});f.accepted[1].secondary_source_id=107;f.accepted[1].generation=7;
  f.spatial={{2,1},{1,3},{1,1},{2,3}};f.Rebuild();
  const auto cpu=RunLifecycleFixture(f);Same(cpu,OracleLifecycle(f.Input()));
  ASSERT_EQ(cpu.occurrences.size(),3u);
  EXPECT_EQ(cpu.occurrences[0].origin,l::Origin::Retained);
  EXPECT_EQ(cpu.occurrences[1].origin,l::Origin::Spatial);EXPECT_EQ(cpu.occurrences[1].source_ordinal,0u);
  EXPECT_EQ(cpu.occurrences[2].origin,l::Origin::Sliding);
  device::Device gpu;
  for(bool reverse:{false,true}) {
    l::HostResult actual;ASSERT_EQ(gpu.Evaluate(f,actual,reverse).status,n::selection::Status::Ok);
    Same(actual,cpu,true);
  }
}
TEST(Type25LifecycleCuda, PersistedIcontLossReacquisitionAndDiscardRetry) {
  Fixture f;f.Retained();f.secondary[0].initial_contact_flag=-1;
  f.positions[20]=2;f.velocities[20]=1800;
  device::Device gpu;l::HostResult lost;
  ASSERT_EQ(gpu.Evaluate(f,lost).status,n::selection::Status::Ok);
  Same(lost,OracleLifecycle(f.Input()));ASSERT_EQ(lost.rows[0].initial_contact_flag,0);
  ASSERT_EQ(lost.rows[0].history.row.irtlm[0],0);
  const auto finished=gpu.Finish(lost.rows);
  const auto native_finished=OracleFinish(lost.rows);
  type25_geometry_test::Same(finished[0].history,native_finished[0].history,true);
  f.accepted[0]=finished[0].history;f.secondary[0].initial_contact_flag=finished[0].initial_contact_flag;
  f.positions[20]=.2;f.velocities[20]=-1800;f.step.time=.001;
  auto actual=lost;
  const auto rejected=gpu.Evaluate(f,actual,false,0);
  EXPECT_EQ(rejected.status,n::selection::Status::CapacityExceeded);EXPECT_TRUE(rejected.count_complete);
  EXPECT_GT(rejected.required_candidates,0u);Same(actual,lost,true);
  ASSERT_EQ(gpu.Evaluate(f,actual).status,n::selection::Status::Ok);
  Same(actual,RunLifecycleFixture(f),true);Same(actual,OracleLifecycle(f.Input()));EXPECT_GT(actual.rows[0].kept_count,0u);
  auto wrongly_reseeded=f;wrongly_reseeded.secondary[0].initial_contact_flag=-1;
  l::HostResult wrong;ASSERT_EQ(gpu.Evaluate(wrongly_reseeded,wrong).status,n::selection::Status::Ok);
  EXPECT_EQ(wrong.rows[0].kept_count,0u);
}
TEST(Type25LifecycleCuda, MalformedLateRowsAndUnknownPrecisionDoNotReplaceResult) {
  Fixture f;device::Device gpu;l::HostResult original;
  ASSERT_EQ(gpu.Evaluate(f,original).status,n::selection::Status::Ok);
  for(unsigned fault=0;fault<3;++fault) {
    auto broken=f;auto out=original;
    if(fault==0)broken.spatial_entries[1]=0;
    if(fault==1)broken.accepted[0].generation++;
    if(fault==2)broken.profile.optcd_response_precision=-1;
    EXPECT_NE(gpu.Evaluate(broken,out).status,n::selection::Status::Ok);Same(out,original,true);
  }
  l::HostResult retry;ASSERT_EQ(gpu.Evaluate(f,retry).status,n::selection::Status::Ok);Same(retry,original,true);
}

TEST(Type25LifecycleCuda, PostForceMarkerNormalizationRemainsAnExplicitDevicePhase) {
  Fixture f;device::Device gpu;l::HostResult selected;
  ASSERT_EQ(gpu.Evaluate(f,selected).status,n::selection::Status::Ok);
  ASSERT_LT(selected.rows[0].history.row.irtlm[0],0);
  const auto actual=gpu.Finish(selected.rows),native=OracleFinish(selected.rows);
  type25_geometry_test::Same(actual[0].history,native[0].history,true);
  EXPECT_LT(selected.rows[0].history.row.irtlm[0],0);
  EXPECT_GT(actual[0].history.row.irtlm[0],0);
}

TEST(Type25LifecycleCuda, MixedTriangleQuadSlidingPreservesThreeSlotNativePhase) {
  device::Device gpu;
  for(bool retained:{false,true}) {
    Fixture f;f.TrianglePair();if(retained){f.Retained();f.positions[18]=5;f.positions[19]=1;}
    l::HostResult actual;ASSERT_EQ(gpu.Evaluate(f,actual).status,n::selection::Status::Ok);
    Same(actual,RunLifecycleFixture(f),true);Same(actual,OracleLifecycle(f.Input()));
    EXPECT_EQ(actual.rows[0].sliding_reference[3],0);
    if(retained)EXPECT_GT(actual.rows[0].sliding_count,0u);
  }
}
TEST(Type25LifecycleCuda, LateRowFailureReportsWholeCountAndPreservesPriorResult) {
  Fixture f;device::Device gpu;l::HostResult original;
  ASSERT_EQ(gpu.Evaluate(f,original).status,n::selection::Status::Ok);
  f.AddSecondary(5,1,.2);f.spatial={{1,1},{2,3}};f.Rebuild();
  f.mains[2].coefficient=std::numeric_limits<double>::max();auto out=original;
  const auto failure=gpu.Evaluate(f,out);
  EXPECT_EQ(failure.status,n::selection::Status::NonfiniteResult);EXPECT_EQ(failure.stage,l::Stage::NewImpact);
  EXPECT_EQ(failure.secondary,1u);EXPECT_TRUE(failure.count_complete);EXPECT_EQ(failure.required_candidates,2u);
  Same(out,original,true);
  f.mains[2].coefficient=400;ASSERT_EQ(gpu.Evaluate(f,out).status,n::selection::Status::Ok);
  Same(out,RunLifecycleFixture(f),true);Same(out,OracleLifecycle(f.Input()));
}
} // namespace type25_lifecycle_test
