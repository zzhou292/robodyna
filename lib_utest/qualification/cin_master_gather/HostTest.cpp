// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "lib_src/elements/mapped_shell/Incidence.h"
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstring>
namespace tl::fea::cin_gather_test {
TEST(CinMasterGather, FullFrozenForceMatchesSharedRepeatedAndPermutedSourceRows) {
  for (unsigned count:{1,2,127,128,129}) for (bool permute:{false,true}) {
    auto serial=old::Population(count);
    if (permute) std::reverse(serial.rows.begin(),serial.rows.end());
    auto actual=serial;
    Workspace scratch;
    for (unsigned step=0;step<3;++step) {
      serial.Begin(step+1);actual.Begin(step+1);
      cin_input_test::Seed(serial);cin_input_test::Seed(actual);
      const auto a=old::FrozenForce(serial),b=Prepared(actual,scratch);
      ASSERT_TRUE(a);cin_input_test::SameReport(a,b);old::SameForce(serial,actual);
      EXPECT_EQ(scratch.summary.mode,gather::Mode::Publish);
      serial.accepted=serial.trial;actual.accepted=actual.trial;
    }
  }
}
TEST(CinMasterGather, OrderedIncidenceRetainsEveryRepeatedSourceSlot) {
  auto p=old::Population(129);
  auto input=p.Input();
  Workspace scratch;
  ASSERT_TRUE(scratch.Initialize(input));
  ASSERT_EQ(scratch.offsets[scratch.view.master_count],4*p.rows.size());
  for (std::uint32_t index=0;index<scratch.view.master_count;++index) {
    const auto begin=scratch.offsets[index],end=scratch.offsets[index+1];
    ASSERT_LT(begin,end);
    for (auto cursor=begin+1;cursor<end;++cursor)
      EXPECT_LT(scratch.incidence[cursor-1],scratch.incidence[cursor]);
  }
  bool repeated=false;
  for (std::uint32_t row=0;row<p.rows.size();++row) {
    if (p.rows[row].masters[2]!=p.rows[row].masters[3]) continue;
    repeated=true;
    const auto node=p.rows[row].masters[2];
    const auto index=std::find(scratch.nodes.begin(),
        scratch.nodes.begin()+scratch.view.master_count,node)-scratch.nodes.begin();
    ASSERT_LT(index,scratch.view.master_count);
    const auto begin=scratch.offsets[index],end=scratch.offsets[index+1];
    const auto first=std::find(scratch.incidence.begin()+begin,
        scratch.incidence.begin()+end,4*row+2);
    ASSERT_NE(first,scratch.incidence.begin()+end);
    ASSERT_NE(first+1,scratch.incidence.begin()+end);
    EXPECT_EQ(*(first+1),4*row+3);
  }
  EXPECT_TRUE(repeated);
}
TEST(CinMasterGather, RejectedPreparedRowDoesNotConsumePoisonedPayload) {
  auto p=old::Population(1);
  const auto input=p.Input();
  const auto force=cin_advance::force_inputs::ForceView(input);
  Workspace scratch;
  ASSERT_TRUE(scratch.Initialize(input));
  auto& rejected=scratch.rows[0];
  rejected.report={cin::StageStatus::InvalidPatch,0,p.rows[0].secondary};
  rejected.secondary_mass=NAN;
  for (unsigned slot=0;slot<4;++slot) {
    rejected.transferred_load.force[slot]={NAN,DBL_MAX,-DBL_MAX};
    rejected.transferred_coefficients.master[slot]={NAN,NAN,NAN};
  }
  const auto node=p.rows[0].masters[0];
  const auto index=std::find(scratch.nodes.begin(),
      scratch.nodes.begin()+scratch.view.master_count,node)-scratch.nodes.begin();
  ASSERT_LT(index,scratch.view.master_count);
  gather::Master value{{11,12,13},14,15,16},before=value;
  EXPECT_FALSE(gather::GatherMaster(input.model,force,scratch.rows.data(),
      scratch.view,index,value));
  EXPECT_EQ(std::memcmp(&value,&before,sizeof(value)),0);
  double numerical=17;
  EXPECT_FALSE(gather::NumericalMass(input.model,force,scratch.rows.data(),numerical));
  EXPECT_EQ(numerical,17);
}
TEST(CinMasterGather, NonzeroIncomingForceCancellationKeepsFrozenBits) {
  auto serial=old::Population(1);
  auto input=serial.Input();
  auto force=cin_advance::force_inputs::ForceView(input);
  ASSERT_TRUE(cin::detail::CheckForceInputs(input.model,force));
  for (std::uint32_t node=0;node<input.model.node_count;++node)
    force.entry_inertia[node]=force.inertia[node];
  cin::detail::PreparedForceRow prepared;
  prepared.report=cin::detail::PrepareForceRow(input.model,force,0,prepared);
  ASSERT_TRUE(prepared.report);
  unsigned selected_slot=4,selected_axis=3;
  double selected=0;
  for (unsigned slot=0;slot<4&&selected_slot==4;++slot) {
    if (std::count(serial.rows[0].masters,serial.rows[0].masters+4,
        serial.rows[0].masters[slot])!=1) continue;
    const auto value=prepared.transferred_load.force[slot];
    const double components[]{value.x,value.y,value.z};
    for (unsigned axis=0;axis<3;++axis) if (components[axis]!=0) {
      selected_slot=slot;selected_axis=axis;selected=components[axis];break;
    }
  }
  ASSERT_LT(selected_slot,4u);
  const auto node=serial.rows[0].masters[selected_slot];
  serial.loads[selected_axis*packet::Nodes+node]=-selected;
  auto actual=serial;
  Workspace scratch;
  const auto a=old::FrozenForce(serial),b=Prepared(actual,scratch);
  ASSERT_TRUE(a);
  cin_input_test::SameReport(a,b);
  old::SameForce(serial,actual);
  EXPECT_EQ(serial.loads[selected_axis*packet::Nodes+node],0);
  EXPECT_FALSE(std::signbit(serial.loads[selected_axis*packet::Nodes+node]));
}
TEST(CinMasterGather, WholeSerialFallbackKeepsFirstErrorAndCompleteFailingRowWrites) {
  for (unsigned fault=0;fault<3;++fault) {
    auto serial=old::Population(2);old::CenterFirstSecondary(serial);old::LatePatch(serial);
    const auto secondary=serial.rows[0].secondary;
    if (fault==0) {
      serial.loads[serial.rows[0].masters[0]]=DBL_MAX;
      serial.loads[secondary]=DBL_MAX/8;
      for (unsigned axis=1;axis<6;++axis) serial.loads[axis*packet::Nodes+secondary]=0;
    } else if (fault==1) {
      serial.trial.back()=DBL_MAX;
      serial.trial[serial.TailOffset()+secondary]=DBL_MAX/2;
    }
    auto actual=serial;Workspace scratch;
    const auto a=old::FrozenForce(serial),b=Prepared(actual,scratch);
    ASSERT_FALSE(a);cin_input_test::SameReport(a,b);old::SameForce(serial,actual);
    EXPECT_EQ(scratch.summary.mode,gather::Mode::SerialCompleted);
    auto retry=old::Population(2),reference=retry;
    ASSERT_TRUE(old::FrozenForce(reference));ASSERT_TRUE(Prepared(retry,scratch));
    EXPECT_EQ(scratch.summary.mode,gather::Mode::Publish);
    old::SameForce(reference,retry);
  }
}
TEST(CinMasterGather, SuccessfulConservativeFallbackPublishesOnce) {
  auto serial=old::Population(129),actual=serial;Workspace scratch;
  ASSERT_TRUE(old::FrozenForce(serial));ASSERT_TRUE(Prepared(actual,scratch,true));
  EXPECT_EQ(scratch.summary.mode,gather::Mode::SerialCompleted);
  old::SameForce(serial,actual);
}
TEST(CinMasterGather, ZeroMassConditionalSavesAndUntouchedChannelsKeepTheirBits) {
  for (double zero:{0.0,-0.0}) {
    auto serial=old::Population(2);
    auto input=serial.Input();auto force=cin_advance::force_inputs::ForceView(input);
    for (std::uint32_t row=0;row<input.model.row_count;++row) {
      const auto node=input.model.rows[row].secondary;
      force.mass[node]=zero;force.inertia[node]=zero;
      force.saved_secondary_mass[row]=.75+row;
      force.saved_secondary_inertia[row]=.125+row;
    }
    for (std::uint32_t node=0;node<input.model.node_count;++node) {
      for (unsigned axis=3;axis<6;++axis) force.loads[axis*input.model.node_count+node]=zero;
      force.rotational_stiffness[node]=zero;
    }
    auto actual=serial;Workspace scratch;
    ASSERT_TRUE(old::FrozenForce(serial));ASSERT_TRUE(Prepared(actual,scratch));
    old::SameForce(serial,actual);
  }
}
TEST(CinMasterGather, RawAndForeignDescriptorsDoNotOptIntoParallelApply) {
  auto p=old::Population(2);auto input=p.Input();Workspace scratch;
  EXPECT_FALSE(gather::Eligible(input.model,input.force_gather));
  ASSERT_TRUE(scratch.Initialize(input));EXPECT_TRUE(gather::Eligible(input.model,scratch.view));
  auto wrong=scratch.view;++wrong.row_count;EXPECT_FALSE(gather::Eligible(input.model,wrong));
  wrong=scratch.view;wrong.source_rows=nullptr;EXPECT_FALSE(gather::Eligible(input.model,wrong));
  wrong=scratch.view;wrong.summary=nullptr;EXPECT_FALSE(gather::Eligible(input.model,wrong));
}
TEST(CinMasterGather, ShellWrapperStillRejectsRepeatedSlotsWithoutOutputWrites) {
  struct Parent { std::uint32_t nodes[4]; } parent{{0,1,2,2}};
  std::uint32_t offsets[4]{91,92,93,94},incidence[4]{81,82,83,84};
  EXPECT_FALSE(mapped_shell::BuildIncidence<4>(&parent,1,3,offsets,4,incidence,4));
  EXPECT_EQ(offsets[0],91);EXPECT_EQ(offsets[3],94);EXPECT_EQ(incidence[0],81);
  parent.nodes[3]=3;
  std::uint32_t complete_offsets[5]{};
  ASSERT_TRUE(mapped_shell::BuildIncidence<4>(&parent,1,4,complete_offsets,5,incidence,4));
  for (unsigned i=0;i<4;++i) EXPECT_EQ(incidence[i],i);
}
} // namespace tl::fea::cin_gather_test

namespace tl::fea::cin_gather_test {
TEST(CinMasterGatherReuse, DependentMasterOverlapIsRejectedBeforeIncidenceWrites) {
  auto p=old::Population(2);auto input=p.Input();
  p.rows[0].masters[0]=p.rows[1].secondary;
  std::vector<std::uint32_t> dense(packet::Nodes+1,19),nodes(8,19),offsets(9,19),incidence(8,19);
  std::uint32_t count=77;
  EXPECT_FALSE(gather::BuildIncidence(input.model,dense.data(),nodes.data(),offsets.data(),incidence.data(),8,count));
  EXPECT_EQ(count,77u);
  for (const auto* values:{&dense,&nodes,&offsets,&incidence})
    for (auto value:*values) EXPECT_EQ(value,19u);
}
}
