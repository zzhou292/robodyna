// SPDX-License-Identifier: AGPL-3.0-or-later
#include "SolidSupport.h"
namespace final_control_test::solid {
TEST(SolidLocalControlHost, FullControlRetainsIdentityAndInitialOverridesOnBothRoutes) {
  for(int foam:{1,2})for(bool analytic:{false,true}) {
    f::HostRig rig;ASSERT_TRUE(rig.Initialize(foam,analytic));f::StageAll(rig.state);
    for(bool initial:{false,true})for(bool operands:{false,true})for(bool valid:{false,true}) {
      SCOPED_TRACE(::testing::Message()<<foam<<":"<<analytic<<":"<<initial<<":"<<operands<<":"<<valid);
      const auto identity=Seed(valid,0x1p54);Compare(rig,identity,initial,operands);
      EXPECT_EQ(rig.state.control.status,s::BatchStatus::Success);
      EXPECT_TRUE(rig.state.control.diagnostics.valid);
      if(initial) {
        EXPECT_EQ(rig.state.control.diagnostics.source_instance_id,rig.state.source_instance_id);
        EXPECT_EQ(rig.state.control.diagnostics.owner_id,rig.state.config.owner.owner_id);
        EXPECT_EQ(rig.state.control.diagnostics.phase,s::BatchPhase::Accepted);
      } else EXPECT_EQ(rig.state.control.diagnostics.source_instance_id,identity.source_instance_id);
    }
  }
}
TEST(SolidLocalControlHost, EarlierOverflowBeatsLaterStatusAndPreservesUnvisitedIdentityThenRepair) {
  f::HostRig rig;ASSERT_TRUE(rig.Initialize());
  f::MeasuredRows<b::Traits18> rows(rig.state,3);
  rows.trial[0].cache.diagnostics.internal_work_increment_j=DBL_MAX;
  rows.trial[1].cache.diagnostics.internal_work_increment_j=DBL_MAX;
  rows.status[2]=9;rig.state.solid18_law90.status[0]=13;f::StageAll(rig.state);
  for(bool initial:{false,true})for(bool operands:{false,true})for(bool valid:{false,true}) {
    Compare(rig,Seed(valid,0.),initial,operands);
    EXPECT_EQ(rig.state.control.status,s::BatchStatus::NonfiniteResult);
    EXPECT_EQ(rig.state.control.family,s::Family::Solid18);EXPECT_EQ(rig.state.control.parent,1u);
    EXPECT_EQ(rig.state.control.diagnostics.parent_count[4],95u);
    EXPECT_EQ(rig.state.control.diagnostics.valid,valid);
  }
  rows.trial[1].cache.diagnostics.internal_work_increment_j=-DBL_MAX;f::StageAll(rig.state);
  Compare(rig,Seed(true,0.),false,true);EXPECT_EQ(rig.state.control.status,s::BatchStatus::ElementFailure);EXPECT_EQ(rig.state.control.parent,2u);
  rows.status[2]=0;rig.state.solid18_law90.status[0]=0;f::StageAll(rig.state);
  Compare(rig,Seed(false,0.),false,true);EXPECT_EQ(rig.state.control.status,s::BatchStatus::Success);
}
TEST(SolidLocalControlHost, EveryFamilyFaultRetainsPartialCountersAndExplicitDestination) {
  f::HostRig rig;ASSERT_TRUE(rig.Initialize());
  int* status[]{rig.state.solid18.status,rig.state.solid24.status,rig.state.solid6z.status,
      rig.state.solid18_law44.status,rig.state.solid18_law90.status};
  std::uint8_t* flags[]{rig.state.solid18.result_valid,rig.state.solid24.result_valid,rig.state.solid6z.result_valid,
      rig.state.solid18_law44.result_valid,rig.state.solid18_law90.result_valid};
  for(unsigned family=0;family<5;++family)for(bool bad_status:{false,true}) {
    f::StageAll(rig.state);if(bad_status)*status[family]=9;else *flags[family]=0;
    for(bool initial:{false,true})for(bool operands:{false,true})Compare(rig,Seed(true,-0.),initial,operands);
    EXPECT_NE(rig.state.control.status,s::BatchStatus::Success);EXPECT_EQ(rig.state.control.parent,0u);
    *status[family]=0;
  }
  f::StageAll(rig.state);const auto retained=Poison();rig.state.control=retained;
  auto separate=retained;separate.diagnostics=Seed(false,.25);
  ASSERT_TRUE(b::MeasureOperandFamily<b::Traits18>(rig.state,separate,0,nullptr));
  f::SameControl(rig.state.control,retained); // private destination does not touch header Control
  EXPECT_NE(separate.diagnostics.parent_count[0],retained.diagnostics.parent_count[0]);
}
template<class Traits> void SeparateFamily(f::HostRig& rig,const b::Control& control) {
  const auto& family=b::FamilyStorage<Traits>(rig.state);
  const auto separate=[&](const void* p,std::size_t bytes) {
    EXPECT_TRUE(fe::trial_identity::Disjoint(&control,sizeof(control),p,bytes));
  };
  separate(family.parents,family.count*sizeof(*family.parents));
  separate(family.status,family.count*sizeof(*family.status));
  separate(family.result_valid,family.count*sizeof(*family.result_valid));
  separate(family.measurement,family.count*sizeof(*family.measurement));
  for(unsigned i=0;i<2;++i)separate(family.slab[i],family.count*sizeof(*family.slab[i]));
}
TEST(SolidLocalControlHost, RealFiveFamilyDeviceLayoutKeepsHeaderControlDisjoint) {
  f::HostRig rig;ASSERT_TRUE(rig.Initialize());
  const auto* header=tl::util::ArenaPointer<b::Storage>(rig.arena.data(),rig.layout.header);
  ASSERT_NE(header,nullptr);
  SeparateFamily<b::Traits18>(rig,header->control);SeparateFamily<b::Traits24>(rig,header->control);
  SeparateFamily<b::Traits6z>(rig,header->control);SeparateFamily<b::Traits18Law44>(rig,header->control);
  SeparateFamily<b::Traits18Law90>(rig,header->control);
}
} // namespace final_control_test::solid
