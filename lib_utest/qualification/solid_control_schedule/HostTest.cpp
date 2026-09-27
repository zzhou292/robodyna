// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
namespace control_schedule_test {
TEST(SolidControlSchedule, OwnsCompleteNativePacketsWithExactFamilyLocalJoin) {
  Fixture f;c::Selection selection;ASSERT_TRUE(Initialize(f,selection));
  ASSERT_EQ(selection.parents().size(),6u);ASSERT_EQ(selection.members().size(),6u);
  EXPECT_EQ(selection.controlled_count(),4u);EXPECT_EQ(selection.packets().size(),5u);
  EXPECT_EQ(selection.members()[0].family,s::Family::Solid18Law90);EXPECT_EQ(selection.members()[0].family_index,0u);
  EXPECT_EQ(selection.members()[1].element_id,202u);EXPECT_EQ(selection.members()[1].family_index,1u);
  EXPECT_EQ(selection.members()[2].element_id,102u);EXPECT_EQ(selection.members()[2].family_index,0u);
  EXPECT_EQ(selection.parents()[1].packet_index,1u);EXPECT_EQ(selection.parents()[1].packet_slot,1u);
  EXPECT_EQ(selection.parents()[2].packet_index,1u);EXPECT_EQ(selection.parents()[2].packet_slot,0u);
  f.members[1]=999;f.source[0].native_property_id=0;f.packets.clear();
  EXPECT_EQ(selection.members()[1].element_id,202u);EXPECT_EQ(selection.packets().size(),5u);
  EXPECT_EQ(selection.Initialize(*f.base.contributions(),{}).status,c::Status::AlreadyInitialized);
}
TEST(SolidControlSchedule, DefaultLegacyHasOnlyFixedHandleAndNoOptionalRows) {
  Fixture f;const auto* selection=f.base.control_selection();ASSERT_NE(selection,nullptr);
  EXPECT_TRUE(selection->prepared());EXPECT_EQ(selection->profile(),c::Profile::LegacyNoStructuralIcontrol);
  EXPECT_EQ(selection->parents().size(),0u);EXPECT_EQ(selection->packets().size(),0u);
  EXPECT_EQ(selection->owned_payload_bytes(),sizeof(c::Selection));EXPECT_EQ(selection->startup_payload_bytes(),sizeof(c::Selection));
  c::Selection invalid;auto input=f.Controls();input.profile=c::Profile::LegacyNoStructuralIcontrol;
  EXPECT_EQ(invalid.Initialize(*f.base.contributions(),input).status,c::Status::InvalidInput);EXPECT_FALSE(invalid.prepared());
}
TEST(SolidControlSchedule, DuplicateUnknownOmittedAndMixedSourceRowsFailBeforePublication) {
  auto reject=[](Fixture& f,c::Status expected){c::Selection selection;EXPECT_EQ(Initialize(f,selection).status,expected);EXPECT_FALSE(selection.prepared());};
  {Fixture f;f.source[1]=f.source[0];reject(f,c::Status::DuplicateIdentity);}
  {Fixture f;f.source[0].element_id=999;reject(f,c::Status::SourceMismatch);}
  {Fixture f;f.source[0].material_id+=1;reject(f,c::Status::SourceMismatch);}
  {Fixture f;f.source.pop_back();reject(f,c::Status::InvalidInput);}
  {Fixture f;f.members[2]=f.members[1];reject(f,c::Status::DuplicateIdentity);}
  {Fixture f;f.members[0]=999;reject(f,c::Status::SourceMismatch);}
  {Fixture f;std::swap(f.members[0],f.members[1]);reject(f,c::Status::SourceMismatch);}
  {Fixture f;f.packets[1].icontrol=0;reject(f,c::Status::SourceMismatch);}
  {Fixture f;for(auto& row:f.source)if(row.element_id==101)row.icontrol=1;reject(f,c::Status::UnsupportedProfile);}
}
TEST(SolidControlSchedule, NativeNftNelAndPartitionRostersCannotHaveHiddenGaps) {
  auto reject=[](Fixture& f){c::Selection selection;EXPECT_FALSE(Initialize(f,selection));EXPECT_FALSE(selection.prepared());};
  {Fixture f;f.packets[2].native_first+=1;reject(f);}
  {Fixture f;f.packets[1].member_count=1;reject(f);}
  {Fixture f;f.packets[0].member_begin=1;reject(f);}
  {Fixture f;f.packets[1].group_id=f.packets[0].group_id;reject(f);}
  {Fixture f;f.partitions[0].member_count=5;reject(f);}
  {Fixture f;f.partitions[0].partition_id=1;reject(f);}
  {Fixture f;c::Selection selection;auto input=f.Controls();input.native_nvsiz=1;
   EXPECT_FALSE(selection.Initialize(*f.base.contributions(),input));EXPECT_FALSE(selection.prepared());}
  Fixture f;f.partitions={{0,0,2,0,3},{1,2,0,3,0},{2,2,3,3,3}};
  for(unsigned n=2;n<5;++n)f.packets[n].native_first-=3;
  c::Selection valid;ASSERT_TRUE(Initialize(f,valid));EXPECT_EQ(valid.packets()[2].partition_index,2u);
}
TEST(SolidControlSchedule, ExactAndOneByteShortSelectionAndCompleteModelCaps) {
  Fixture f;c::Budget forecast;ASSERT_TRUE(c::Selection::Forecast(f.Controls(),6,{},forecast));
  c::Limits limits;limits.max_host_bytes=forecast.startup_bytes;
  c::Selection exact;ASSERT_TRUE(exact.Initialize(*f.base.contributions(),f.Controls(),limits));
  EXPECT_EQ(exact.owned_payload_bytes(),forecast.owned_bytes);EXPECT_EQ(exact.startup_payload_bytes(),forecast.startup_bytes);
  --limits.max_host_bytes;c::Selection short_cap;
  EXPECT_EQ(short_cap.Initialize(*f.base.contributions(),f.Controls(),limits).status,c::Status::ResourceLimit);EXPECT_FALSE(short_cap.prepared());
  s::Model model;ASSERT_TRUE(model.Initialize(f.Domain(),f.ControlledInput()));
  s::ModelLimits model_limits;model_limits.max_host_bytes=model.startup_payload_bytes();
  s::Model model_exact;ASSERT_TRUE(model_exact.Initialize(f.Domain(),f.ControlledInput(),model_limits));
  EXPECT_EQ(model_exact.owned_payload_bytes(),model.owned_payload_bytes());
  --model_limits.max_host_bytes;s::Model model_short;
  EXPECT_EQ(model_short.Initialize(f.Domain(),f.ControlledInput(),model_limits).status,s::ModelStatus::ResourceLimit);EXPECT_FALSE(model_short.prepared());
}
TEST(SolidControlSchedule, CountOnlyAdmissionPrecedesInvalidBorrowedReads) {
  Fixture f;auto in=f.Controls();in.parents={reinterpret_cast<const c::SourceParent*>(1),SIZE_MAX};
  c::Budget untouched{17,19};
  EXPECT_EQ(c::Selection::Forecast(in,6,{},untouched).status,c::Status::ResourceLimit);
  EXPECT_EQ(untouched.owned_bytes,17u);EXPECT_EQ(untouched.startup_bytes,19u);
}
TEST(SolidControlSchedule, ModelIdentityIncludesSelectionUnitsAndEveryNativePacketOrder) {
  Fixture f;s::Model first,clone;ASSERT_TRUE(first.Initialize(f.Domain(),f.ControlledInput()));
  ASSERT_TRUE(clone.Initialize(f.Domain(),f.ControlledInput()));EXPECT_TRUE(first.Matches(clone));EXPECT_FALSE(first.Matches(f.base));
  {auto in=f.ControlledInput();in.controls.units={1,1,1};s::Model other;ASSERT_TRUE(other.Initialize(f.Domain(),in));EXPECT_FALSE(first.Matches(other));}
  {auto in=f.ControlledInput();in.controls.native_nvsiz=64;s::Model other;ASSERT_TRUE(other.Initialize(f.Domain(),in));EXPECT_FALSE(first.Matches(other));}
  {std::swap(f.members[1],f.members[2]);s::Model other;ASSERT_TRUE(other.Initialize(f.Domain(),f.ControlledInput()));EXPECT_FALSE(first.Matches(other));std::swap(f.members[1],f.members[2]);}
  {for(auto& row:f.source)if(row.element_id==102||row.element_id==202)row.icontrol=0;f.packets[1].icontrol=0;
   s::Model other;ASSERT_TRUE(other.Initialize(f.Domain(),f.ControlledInput()));EXPECT_FALSE(first.Matches(other));}
}
TEST(SolidControlSchedule, CurrentResidentCannotSilentlyExecuteSourceDeclaredControls) {
  Fixture f;s::Model model;ASSERT_TRUE(model.Initialize(f.Domain(),f.ControlledInput()));
  s::batch_detail::ArenaLayout output;const auto report=s::batch_detail::Plan({},model,output);
  EXPECT_EQ(report.status,s::BatchStatus::InvalidInput);
  EXPECT_STREQ(report.message,"Source-declared structural solid controls require a qualified controlled resident");
}
} // namespace control_schedule_test
