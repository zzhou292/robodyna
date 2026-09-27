// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeExpected.h"
using namespace law90_control_test;
TEST(Law90ControlledDistortion, ReaderContactBulkRemainsDistinctFromUpdatedBulk) {
  for(const auto units:{d::UnitScale{1,1,1},d::UnitScale{.001,1000,1}})for(unsigned mode=0;mode<3;++mode) {
    auto input=law90_test::OriginalBlankHuInput();const auto curve=law90_test::OriginalBlankHuCurve();
    if(mode==1)input.contact_modulus_pa=0;if(mode==2)input.card_young_pa=300e9;
    law::PreparedMaterial source;ASSERT_EQ(law::PrepareSI(input,curve,source),law::Status::Ok);
    c::Material control;ASSERT_EQ(c::PrepareMaterial(source,units,control),d::Status::Success);
    const auto native=law90_point_test::NativePrepared(WorkingInput(input,units),curve);
    const auto& slots=control.slots();
    EXPECT_DOUBLE_EQ(slots.pm21_poisson_ratio,native[4]);
    EXPECT_DOUBLE_EQ(slots.pm22_shear_pa,native[28]);EXPECT_DOUBLE_EQ(slots.pm32_bulk_pa,native[29]);
    EXPECT_DOUBLE_EQ(slots.pm100_reader_contact_bulk_pa,native[6]);
    EXPECT_DOUBLE_EQ(slots.pm107_control_pa,2*::fmax(native[29],native[6]));
    if(mode==0)EXPECT_NE(slots.pm32_bulk_pa,slots.pm100_reader_contact_bulk_pa);
  }
}
TEST(Law90ControlledDistortion, FullNativeRecurrenceBothWorkingUnitsAndSourceOrientations) {
  for(const auto units:{d::UnitScale{1,1,1},d::UnitScale{.001,1000,1}})for(bool reflected:{false,true}) {
    auto input=law90_force_test::Distorted();const auto material=law90_force_test::Material(true);
    input.density_kg_m3=material.reader().density_kg_m3;
    if(reflected)for(auto& x:input.position_m)x.x=-x.x;
    const auto reference=Reference(input,material,units);
    const auto curve=law90_test::OriginalBlankHuCurve();
    const auto native_material=law90_point_test::NativePrepared(WorkingInput(law90_test::OriginalBlankHuInput(),units),curve);
    NativeState native(native_material[1]);c::Scratch scratch;
    ASSERT_EQ(c::PrepareInitial(reference,{0,0,0},scratch),d::Status::Success);
    s::PrescribedInterval initial;
    for(unsigned n=0;n<8;++n)initial.position_endpoint_m[n]=input.position_m[n];
    c::Result result;ASSERT_EQ(c::Complete(scratch,scratch.activity.triggers_native_batch!=0,result),d::Status::Success);
    Compare(result,Native(native_material.data(),curve,input,initial,units,true,native));
    unsigned damping_responses=0;
    for(unsigned step=1;step<=32;++step) {
      SCOPED_TRACE(step);auto interval=Path(input,step);
      MatchBase(result.proposed_history.native_history(),interval);
      ASSERT_EQ(c::PrepareCandidate(reference,result.proposed_history,interval,scratch),d::Status::Success);
      c::Result next;ASSERT_EQ(c::Complete(scratch,scratch.activity.triggers_native_batch!=0,next),d::Status::Success);
      const auto expected=Native(native_material.data(),curve,input,interval,units,false,native);Compare(next,expected);
      EXPECT_DOUBLE_EQ(next.nodal_raw_stiffness_n_m,
          scratch.force.staged.diagnostics.raw_stiffness_n_m*Factors(units).base.stiffness);
      if(next.distortion_work_increment_j!=0)++damping_responses;
      result=next;
    }
    EXPECT_GT(damping_responses,0u);
  }
}
TEST(Law90ControlledDistortion, LastGaussVolumeAndStiffnessAreNotCenterOrSum) {
  auto input=law90_force_test::Distorted();const auto material=law90_force_test::Material(true);
  input.density_kg_m3=material.reader().density_kg_m3;
  const auto reference=Reference(input,material,{1,1,1});c::Scratch scratch;
  ASSERT_EQ(c::PrepareInitial(reference,{0,0,0},scratch),d::Status::Success);
  const auto& before=scratch.force.staged;const auto& prepared=scratch.distortion;
  EXPECT_DOUBLE_EQ(prepared.input.raw_stiffness,before.point[7].material.raw_stiffness_n_m);
  EXPECT_NE(prepared.input.raw_stiffness,before.diagnostics.raw_stiffness_n_m);
  EXPECT_DOUBLE_EQ(prepared.parameters.length,::pow(before.point[7].current_volume_m3,1.0/3.0));
  EXPECT_NE(prepared.parameters.length,::pow(reference.source().geometry().center_volume_m3,1.0/3.0));
}
TEST(Law90ControlledDistortion, InvalidSourceUnitsAndLateForceAreFailureAtomic) {
  const auto material=law90_force_test::Material(true);c::Material control;
  ASSERT_EQ(c::PrepareMaterial(material,{1,1,1},control),d::Status::Success);
  const auto saved=law90_test::Bytes(control);
  EXPECT_EQ(c::PrepareMaterial(material,{.01,1,1},control),d::Status::UnsupportedProfile);
  EXPECT_EQ(law90_test::Bytes(control),saved);
  auto input=law90_force_test::Distorted();input.density_kg_m3=material.reader().density_kg_m3;
  const auto reference=Reference(input,material,{1,1,1});c::Scratch scratch;c::Result accepted;
  ASSERT_EQ(c::PrepareInitial(reference,{0,0,0},scratch),d::Status::Success);
  ASSERT_EQ(c::Complete(scratch,false,accepted),d::Status::Success);
  auto trial=accepted;const auto before=law90_test::Bytes(trial);
  auto interval=Path(input,1);interval.position_endpoint_m[0].x=std::numeric_limits<double>::quiet_NaN();
  EXPECT_NE(c::PrepareCandidate(reference,accepted.proposed_history,interval,scratch),d::Status::Success);
  EXPECT_EQ(c::Complete(scratch,false,trial),d::Status::InvalidInput);
  EXPECT_EQ(law90_test::Bytes(trial),before);
  const auto different_units=Reference(input,material,{.001,1000,1});
  EXPECT_EQ(c::PrepareCandidate(different_units,accepted.proposed_history,Path(input,1),scratch),d::Status::InvalidInput);
}

TEST(Law90ControlledDistortion, AmbiguousNativeCutoffSentinelIsNotSilentlyRescaled) {
  auto input=law90_test::OriginalBlankHuInput();input.tension_cutoff_pa=0;
  law::PreparedMaterial material;
  ASSERT_EQ(law::PrepareSI(input,law90_test::OriginalBlankHuCurve(),material),law::Status::Ok);
  c::Material result;ASSERT_EQ(c::PrepareMaterial(material,{1,1,1},result),d::Status::Success);
  const auto saved=law90_test::Bytes(result);
  EXPECT_EQ(c::PrepareMaterial(material,{.001,1000,1},result),d::Status::UnsupportedProfile);
  EXPECT_EQ(law90_test::Bytes(result),saved);
}
