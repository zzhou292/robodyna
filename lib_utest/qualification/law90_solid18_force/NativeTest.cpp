// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeSupport.h"
using namespace law90_force_test;
TEST(Law90Solid18Native, CompleteCallerConstructorAndIndependentHistory) {
  law90_point_test::ToyCurve toy;
  for(bool original:{true,false}) {
    const auto input=original ? law90_test::OriginalInput() : law90_point_test::ToyInput();
    const auto curve=original ? law90_test::OriginalCurve() : toy.view();
    law::PreparedMaterial material;ASSERT_EQ(law::PrepareSI(input,curve,material),law::Status::Ok);
    const auto prepared=law90_point_test::NativePrepared(input,curve);
    NativeCaller native(prepared[1]);law::CallerResult accepted;
    unsigned cursor_changes=0;double maximum_q=0,maximum_work=0;
    EXPECT_EQ(native.history[16],material.reader().reference_density_kg_m3);
    for(unsigned step=0;step<=320;++step) {
      SCOPED_TRACE(step);const auto packet=CallerPath(step);
      const int old_cursor[3]{native.cursor[0],native.cursor[1],native.cursor[2]};
      AdvanceNative(prepared.data(),curve,packet,native);ASSERT_EQ(native.status,0);
      law::CallerResult trial;
      ASSERT_EQ(step ? law::UpdateCallerSI(material,accepted.history,packet,trial) :
          law::InitializeCallerSI(material,packet,trial),law::PointStatus::Ok);
      ASSERT_TRUE(CallerAgreement(CallerValues(trial),native.values.data()));
      EXPECT_EQ(trial.density_compression,trial.history.density_kg_m3/prepared[1]-1);
      for(unsigned k=0;k<3;++k) {
        ASSERT_EQ(trial.history.point.cursor[k],static_cast<unsigned>(native.cursor[k]));
        cursor_changes+=old_cursor[k]!=native.cursor[k];
      }
      maximum_q=std::max(maximum_q,trial.history.bulk_pressure_pa);
      maximum_work=std::max(maximum_work,std::abs(trial.internal_work_j));
      accepted=trial;
    }
    EXPECT_GT(cursor_changes,0u);EXPECT_GT(maximum_q,0);EXPECT_GT(maximum_work,0);
  }
}
TEST(Law90Solid18Native, StorageFloorAndActualPressureVolumeWork) {
  const auto input=law90_test::OriginalInput();const auto curve=law90_test::OriginalCurve();
  const auto material=Material();const auto prepared=law90_point_test::NativePrepared(input,curve);
  for(double volume:{1e-22,1e-20,1e-6}) {
    NativeCaller native(prepared[1]);law::CallerResult accepted;
    auto packet=CallerPath(0);packet.current_volume_m3=packet.storage_volume_m3=volume;
    AdvanceNative(prepared.data(),curve,packet,native);ASSERT_EQ(native.status,0);
    ASSERT_EQ(law::InitializeCallerSI(material,packet,accepted),law::PointStatus::Ok);
    ASSERT_TRUE(CallerAgreement(CallerValues(accepted),native.values.data()));
    // Explicit prescribed EINT/rho/Q state, independently seeded on both sides.
    accepted.history.internal_energy_density_j_m3=1234;native.history[17]=1234;
    accepted.history.density_kg_m3=input.density_kg_m3*1.05;native.history[16]=accepted.history.density_kg_m3;
    accepted.history.bulk_pressure_pa=42;native.history[18]=42;
    packet=CallerPath(40);packet.storage_volume_m3=volume;packet.current_volume_m3=.8*volume;
    AdvanceNative(prepared.data(),curve,packet,native);ASSERT_EQ(native.status,0);
    law::CallerResult trial;ASSERT_EQ(law::UpdateCallerSI(material,accepted.history,packet,trial),law::PointStatus::Ok);
    ASSERT_TRUE(CallerAgreement(CallerValues(trial),native.values.data()));
    EXPECT_NE(trial.volume_increment_m3,packet.current_volume_m3-packet.storage_volume_m3);
  }
}
TEST(Law90Solid18Native, FullElementRotatedCompressionUnloadingAndPointCursors) {
  law90_point_test::ToyCurve curve;const auto input_material=ElementToyInput();
  law::PreparedMaterial material;ASSERT_EQ(law::PrepareSI(input_material,curve.view(),material),law::Status::Ok);
  const auto prepared=law90_point_test::NativePrepared(input_material,curve.view());
  for(bool flipped:{false,true}) {
    auto input=Distorted();if(flipped)for(auto& p:input.position_m)p.x=-p.x;
    f::Reference reference;ASSERT_EQ(f::InitializeReference90(input,reference),s::Status::Success);
    f::ForceTrial accepted;ASSERT_EQ(f::InitializeForce90(reference,material,{3,-1,2},accepted),s::Status::Success);
    s::PrescribedInterval virgin;
    for(unsigned n=0;n<8;++n){virgin.position_endpoint_m[n]=input.position_m[n];virgin.velocity_midpoint_m_s[n]={3,-1,2};}
    NativeForce native(input.density_kg_m3);AdvanceNative(prepared.data(),curve.view(),input,virgin,true,native);
    ASSERT_EQ(native.status,0);ASSERT_TRUE(ForceAgreement(ForceValues(accepted),native));CheckCursors(accepted,native);
    for(unsigned step=1;step<=160;++step) {
      SCOPED_TRACE(step);auto interval=Path(input,step);MatchBase(accepted.proposed_history,interval);
      AdvanceNative(prepared.data(),curve.view(),input,interval,false,native);ASSERT_EQ(native.status,0);
      f::ForceTrial trial;ASSERT_EQ(f::EvaluateForce90(reference,accepted.proposed_history,interval,material,trial),s::Status::Success);
      ASSERT_TRUE(ForceAgreement(ForceValues(trial),native));CheckCursors(trial,native);accepted=trial;
    }
  }
}
TEST(Law90Solid18Native, AgreementRejectsDimensionAndStatePerturbations) {
  const auto material=Material();const auto input=Cube();f::Reference reference;
  ASSERT_EQ(f::InitializeReference90(input,reference),s::Status::Success);
  f::ForceTrial accepted;ASSERT_EQ(f::InitializeForce90(reference,material,{},accepted),s::Status::Success);
  const auto prepared=law90_point_test::NativePrepared(law90_test::OriginalInput(),law90_test::OriginalCurve());
  s::PrescribedInterval virgin;for(unsigned n=0;n<8;++n)virgin.position_endpoint_m[n]=input.position_m[n];
  NativeForce native(input.density_kg_m3);AdvanceNative(prepared.data(),material.curve(),input,virgin,true,native);
  ASSERT_EQ(native.status,0);const auto values=ForceValues(accepted);ASSERT_TRUE(ForceAgreement(values,native));
  for(unsigned index:{0u,3u,16u,28u,31u,32u,35u,37u,38u,39u,320u,327u,330u,331u,333u,356u}) {
    auto perturbed=values;perturbed[index]+=1e-5*std::max(1.,std::abs(values[index]));
    EXPECT_FALSE(ForceAgreement(perturbed,native))<<index;
  }
}

TEST(Law90Solid18Native, ExactDefaultHistoryAllocationHasNoPlasticityPlaceholders) {
  int tags[6];law90_force_tags(tags);
  const int expected[6]{0,0,1,0,0,1};
  for(unsigned k=0;k<6;++k)EXPECT_EQ(tags[k],expected[k])<<k;
}
