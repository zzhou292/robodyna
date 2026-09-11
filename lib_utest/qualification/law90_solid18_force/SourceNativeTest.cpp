// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeSupport.h"
#include "lib_utest/qualification/law90_solid18_reference/SourceFixture.h"
using namespace law90_force_test;
TEST(Law90Solid18SourceForce, All1345OriginalIndependentEightPointHistories) {
  const auto material=Material();const auto curve=law90_test::OriginalCurve();
  const auto prepared=law90_point_test::NativePrepared(law90_test::OriginalInput(),curve);
  unsigned observed=0;
  for(unsigned row=0;row<law90_reference_test::fixture::element_count;++row) {
    const auto input=law90_reference_test::Original(row);SCOPED_TRACE(input.source_element_id);
    f::Reference reference;ASSERT_EQ(f::InitializeReference90(input,reference),s::Status::Success);
    f::ForceTrial accepted;ASSERT_EQ(f::InitializeForce90(reference,material,{15.6464,0,0},accepted),s::Status::Success);
    s::PrescribedInterval virgin;
    for(unsigned n=0;n<8;++n){virgin.position_endpoint_m[n]=input.position_m[n];virgin.velocity_midpoint_m_s[n]={15.6464,0,0};}
    NativeForce native(input.density_kg_m3);AdvanceNative(prepared.data(),curve,input,virgin,true,native);
    ASSERT_EQ(native.status,0);ASSERT_TRUE(ForceAgreement(ForceValues(accepted),native));CheckCursors(accepted,native);++observed;
    for(unsigned step=1;step<=8;++step) {
      SCOPED_TRACE(step);auto interval=Path(input,step,1e-6,.03,true);MatchBase(accepted.proposed_history,interval);
      AdvanceNative(prepared.data(),curve,input,interval,false,native);ASSERT_EQ(native.status,0);
      f::ForceTrial trial;ASSERT_EQ(f::EvaluateForce90(reference,accepted.proposed_history,interval,material,trial),s::Status::Success);
      ASSERT_TRUE(ForceAgreement(ForceValues(trial),native));CheckCursors(trial,native);accepted=trial;++observed;
    }
  }
  RecordProperty("original_elements",1345);RecordProperty("native_force_packets",observed);
  EXPECT_EQ(observed,1345u*9u);
}
