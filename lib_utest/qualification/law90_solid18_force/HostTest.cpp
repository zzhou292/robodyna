// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
#include <gtest/gtest.h>
using namespace law90_force_test;
TEST(Law90Solid18Force, ConstructorKeepsEightActualHistoriesAndZeroEpoch) {
  const auto material=Material();const auto input=Distorted();
  f::Reference reference;ASSERT_EQ(f::InitializeReference90(input,reference),s::Status::Success);
  const auto before=Bytes(reference);
  f::ForceTrial initial;ASSERT_EQ(f::InitializeForce90(reference,material,{3,-1,2},initial),s::Status::Success);
  EXPECT_TRUE(initial.proposed_history.prepared());
  EXPECT_EQ(initial.proposed_history.stamp().sample_index,0u);
  EXPECT_EQ(initial.proposed_history.stamp().time_s,0);
  EXPECT_GT(initial.diagnostics.minimum_unscaled_dt_s,0);
  EXPECT_GT(initial.diagnostics.raw_stiffness_n_m,0);
  for(unsigned ip=0;ip<8;++ip) {
    const auto& p=initial.point[ip];
    EXPECT_EQ(p.storage_volume_m3,reference.geometry().point[ip].initial_volume_m3);
    EXPECT_EQ(p.material.history.point.unloading_factor,1);
    EXPECT_EQ(p.material.point.active,1);
    EXPECT_EQ(p.material.point.maximum_viscosity_pa_s,0);
  }
  EXPECT_EQ(Bytes(reference),before);
  RecordProperty("history_bytes",int(sizeof(f::History)));
  RecordProperty("trial_bytes",int(sizeof(f::ForceTrial)));
  RecordProperty("scratch_bytes",int(sizeof(f::ForceScratch)));
}
TEST(Law90Solid18Force, CompressionRotationAndUnloadingCarryNativeState) {
  law90_point_test::ToyCurve curve;law::PreparedMaterial material;
  ASSERT_EQ(law::PrepareSI(ElementToyInput(),curve.view(),material),law::Status::Ok);
  const auto input=Distorted();
  f::Reference reference;ASSERT_EQ(f::InitializeReference90(input,reference),s::Status::Success);
  f::ForceTrial accepted;ASSERT_EQ(f::InitializeForce90(reference,material,{3,-1,2},accepted),s::Status::Success);
  double peak=0;unsigned changed_cursor=0;
  for(unsigned step=1;step<=80;++step) {
    auto interval=Path(input,step);MatchBase(accepted.proposed_history,interval);
    f::ForceTrial trial;
    ASSERT_EQ(f::EvaluateForce90(reference,accepted.proposed_history,interval,material,trial),s::Status::Success)<<step;
    EXPECT_EQ(trial.proposed_history.stamp().sample_index,step);
    for(unsigned ip=0;ip<8;++ip) {
      const auto& h=trial.proposed_history.data().point[ip].point;
      peak=std::max(peak,h.strain_norm);
      for(unsigned k=0;k<3;++k)changed_cursor+=h.cursor[k]!=accepted.proposed_history.data().point[ip].point.cursor[k];
      EXPECT_GE(h.maximum_path_energy_pa,h.path_energy_pa);
      EXPECT_EQ(h.unloading_factor,1);
      EXPECT_EQ(trial.point[ip].storage_volume_m3,accepted.point[ip].storage_volume_m3);
    }
    accepted=trial;
  }
  EXPECT_GT(peak,.2);EXPECT_GT(changed_cursor,0u);
  EXPECT_GT(accepted.proposed_history.data().point[0].point.maximum_path_energy_pa,0);
}
TEST(Law90Solid18Force, CallerUsesActualReturnedSoundAndPreservesRejectedDestination) {
  law90_point_test::ToyCurve curve;law::PreparedMaterial material;
  ASSERT_EQ(law::PrepareSI(law90_point_test::ToyInput(),curve.view(),material),law::Status::Ok);
  law::CallerResult accepted;
  ASSERT_EQ(law::InitializeCallerSI(material,CallerPath(0),accepted),law::PointStatus::Ok);
  auto input=CallerPath(50);law::CallerResult trial;
  ASSERT_EQ(law::UpdateCallerSI(material,accepted.history,input,trial),law::PointStatus::Ok);
  EXPECT_EQ(trial.density_compression,trial.history.density_kg_m3/material.reader().reference_density_kg_m3-1);
  EXPECT_EQ(trial.point.history.cursor[0],trial.history.point.cursor[0]);
  EXPECT_GT(trial.point.sound_speed_m_s,0);
  EXPECT_NE(trial.point.tangent_factor,1);
  const auto before=Bytes(trial);
  input.dt_s=0;EXPECT_NE(law::UpdateCallerSI(material,accepted.history,input,trial),law::PointStatus::Ok);
  EXPECT_EQ(Bytes(trial),before);
  input=CallerPath(50);input.engineering_rate_per_s[5]=std::numeric_limits<double>::infinity();
  EXPECT_NE(law::UpdateCallerSI(material,accepted.history,input,trial),law::PointStatus::Ok);
  EXPECT_EQ(Bytes(trial),before);
  ASSERT_EQ(law::UpdateCallerSI(material,accepted.history,CallerPath(50),trial),law::PointStatus::Ok);
}
TEST(Law90Solid18Force, CompleteReferenceAndBorrowedMaterialIdentityRequired) {
  const auto material=Material();auto input=Cube();f::Reference reference;
  ASSERT_EQ(f::InitializeReference90(input,reference),s::Status::Success);
  f::ForceTrial accepted;ASSERT_EQ(f::InitializeForce90(reference,material,{},accepted),s::Status::Success);
  const auto before=Bytes(accepted);auto interval=Path(input,1);
  input.source_element_id++;f::Reference foreign;
  ASSERT_EQ(f::InitializeReference90(input,foreign),s::Status::Success);
  EXPECT_NE(f::EvaluateForce90(foreign,accepted.proposed_history,interval,material,accepted),s::Status::Success);
  EXPECT_EQ(Bytes(accepted),before);
  auto p=law90_test::OriginalInput();p.reference_density_kg_m3=p.density_kg_m3*2;
  law::PreparedMaterial foreign_material;ASSERT_EQ(law::PrepareSI(p,law90_test::OriginalCurve(),foreign_material),law::Status::Ok);
  EXPECT_NE(f::EvaluateForce90(reference,accepted.proposed_history,interval,foreign_material,accepted),s::Status::Success);
  EXPECT_EQ(Bytes(accepted),before);
  auto values=accepted.proposed_history.data();values.point[7].point.cursor[2]=UINT32_MAX;
  f::History history=accepted.proposed_history;const auto history_before=Bytes(history);
  EXPECT_NE(f::PreparePrescribedHistory90(reference,material,values,{},history),s::Status::Success);
  EXPECT_EQ(Bytes(history),history_before);
}
TEST(Law90Solid18Force, FinalPointOverflowRollsBackAndRetryPublishesOnce) {
  const auto material=Material();auto input=Cube();
  for(auto& p:input.position_m){p.x*=10;p.y*=10;p.z*=10;}
  f::Reference reference;ASSERT_EQ(f::InitializeReference90(input,reference),s::Status::Success);
  f::ForceTrial initial;ASSERT_EQ(f::InitializeForce90(reference,material,{},initial),s::Status::Success);
  auto values=initial.proposed_history.data();values.point[7].internal_energy_density_j_m3=std::numeric_limits<double>::max();
  f::History failing;ASSERT_EQ(f::PreparePrescribedHistory90(reference,material,values,{},failing),s::Status::Success);
  f::ForceTrial destination=initial;const auto before=Bytes(destination);
  const auto interval=Path(input,3);auto first=interval;first.base_time_s=0;first.sample_index=1;
  EXPECT_NE(f::EvaluateForce90(reference,failing,first,material,destination),s::Status::Success);
  EXPECT_EQ(Bytes(destination),before);
  f::ForceScratch scratch;
  EXPECT_NE(f::EvaluateForce90Scratch(reference,failing,first,material,scratch),s::Status::Success);
  EXPECT_GT(scratch.next.point[6].point.strain_norm,0); // Earlier native visits completed.
  ASSERT_EQ(f::EvaluateForce90(reference,initial.proposed_history,first,material,destination),s::Status::Success);
  EXPECT_EQ(destination.proposed_history.stamp().sample_index,1u);
  EXPECT_NE(Bytes(destination),before);
}
TEST(Law90Solid18Force, CurrentInversionAndEpochReplayRejectAtomically) {
  const auto material=Material();const auto input=Cube();f::Reference reference;
  ASSERT_EQ(f::InitializeReference90(input,reference),s::Status::Success);
  f::ForceTrial initial;ASSERT_EQ(f::InitializeForce90(reference,material,{},initial),s::Status::Success);
  auto interval=Path(input,1);f::ForceTrial result=initial;const auto before=Bytes(result);
  interval.sample_index=0;
  EXPECT_NE(f::EvaluateForce90(reference,initial.proposed_history,interval,material,result),s::Status::Success);
  EXPECT_EQ(Bytes(result),before);
  interval=Path(input,1);for(auto& p:interval.position_endpoint_m)p.x=-p.x;
  EXPECT_NE(f::EvaluateForce90(reference,initial.proposed_history,interval,material,result),s::Status::Success);
  EXPECT_EQ(Bytes(result),before);
  interval=Path(input,1);
  ASSERT_EQ(f::EvaluateForce90(reference,initial.proposed_history,interval,material,result),s::Status::Success);
}

TEST(Law90Solid18Force, ActualBlankHuDensityRawCurveAndLatePointRollback) {
  const auto material=Material(true);auto input=Cube();
  input.density_kg_m3=material.reader().density_kg_m3;
  for(auto& p:input.position_m){p.x*=10;p.y*=10;p.z*=10;}
  f::Reference reference;ASSERT_EQ(f::InitializeReference90(input,reference),s::Status::Success);
  f::ForceTrial initial;ASSERT_EQ(f::InitializeForce90(reference,material,{},initial),s::Status::Success);
  EXPECT_EQ(material.reader().loading_flag,1);
  EXPECT_EQ(material.reader().curve_scale,1e6);
  auto values=initial.proposed_history.data();
  values.point[7].internal_energy_density_j_m3=std::numeric_limits<double>::max();
  f::History failing;ASSERT_EQ(f::PreparePrescribedHistory90(reference,material,values,{},failing),s::Status::Success);
  auto interval=Path(input,3);interval.base_time_s=0;interval.sample_index=1;
  f::ForceTrial destination=initial;const auto before=Bytes(destination);
  EXPECT_NE(f::EvaluateForce90(reference,failing,interval,material,destination),s::Status::Success);
  EXPECT_EQ(Bytes(destination),before);
  f::ForceScratch scratch;
  EXPECT_NE(f::EvaluateForce90Scratch(reference,failing,interval,material,scratch),s::Status::Success);
  EXPECT_GT(scratch.next.point[6].point.instantaneous_quasistatic_energy_pa,0);
  ASSERT_EQ(f::EvaluateForce90(reference,initial.proposed_history,interval,material,destination),s::Status::Success);
  for(unsigned ip=0;ip<8;++ip) {
    const auto& point=destination.point[ip].material;
    EXPECT_EQ(point.point.tangent_factor,1e20/material.updated().young_pa);
    EXPECT_EQ(point.history.point.strain_norm,0);
    EXPECT_EQ(point.history.point.path_energy_pa,0);
    EXPECT_GT(point.history.point.instantaneous_quasistatic_energy_pa,0);
  }
  EXPECT_EQ(destination.proposed_history.stamp().sample_index,1u);
}
