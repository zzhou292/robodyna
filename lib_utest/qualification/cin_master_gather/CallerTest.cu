// SPDX-License-Identifier: AGPL-3.0-or-later
#include "DeviceFixture.cuh"
#include <algorithm>
#include <cfloat>
#include <cmath>
namespace tl::fea::cin_gather_test {
TEST(CinMasterGatherCuda,FrozenCompleteCallerMatchesSharedRepeatedRowsAndThreeIntervals) {
  for (unsigned count:{1,2,127,128,129}) for (bool groups:{false,true}) for (bool capture:{false,true}) {
    SCOPED_TRACE(count);
    SCOPED_TRACE(groups);
    SCOPED_TRACE(capture);
    auto serial=old::Population(count,groups,capture);
    if (capture) serial.structural={NodalCinStructuralProfile::NativeOrdinaryRigidTrace,.8,true};
    auto actual=serial;
    for (unsigned step=0;step<3;++step) {
      SCOPED_TRACE(step);
      serial.Begin(step+1);actual.Begin(step+1);
      cin_input_test::Seed(serial);cin_input_test::Seed(actual);
      const auto accepted=actual.accepted;
      RunPair(serial,actual,capture);
      ASSERT_EQ(serial.control.status,NodalStatus::Ok)<<serial.control.node;
      packet::SameSuccessfulPacket(serial,actual);old::SamePatches(serial,actual);
      packet::SameDoubles(actual.accepted,accepted);
      serial.Accept();actual.Accept();
    }
  }
}
TEST(CinMasterGatherCuda,RawDescriptorKeepsCompleteSerialCaller) {
  for (unsigned count:{1,129}) {
    SCOPED_TRACE(count);
    auto serial=old::Population(count,true,true),actual=serial;
    packet::DevicePacket before(serial,true,true),after(actual,true,true);
    before.RunWith(LaunchFrozen);
    after.RunWith(cin_advance::Launch);
    before.Download(serial);
    after.Download(actual);
    ASSERT_EQ(serial.control.status,NodalStatus::Ok)<<serial.control.node;
    packet::SameSuccessfulPacket(serial,actual);
    old::SamePatches(serial,actual);
  }
}
TEST(CinMasterGatherCuda,NonzeroIncomingForceCancellationKeepsFrozenBits) {
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
  RunPair(serial,actual,false);
  ASSERT_EQ(serial.control.status,NodalStatus::Ok)<<serial.control.node;
  packet::SameSuccessfulPacket(serial,actual);
  old::SamePatches(serial,actual);
  EXPECT_EQ(serial.loads[selected_axis*packet::Nodes+node],0);
  EXPECT_FALSE(std::signbit(serial.loads[selected_axis*packet::Nodes+node]));
}
TEST(CinMasterGatherCuda,PostAddFailurePrecedesLateLeafWithAllFailingRowWritesAndRetry) {
  for (unsigned fault=0;fault<3;++fault) {
    auto serial=old::Population(129,true,true);old::CenterFirstSecondary(serial);old::LatePatch(serial);
    const auto secondary=serial.rows[0].secondary;
    if (fault==0) {
      serial.loads[serial.rows[0].masters[0]]=DBL_MAX;
      serial.loads[secondary]=DBL_MAX/8;
      for (unsigned axis=1;axis<6;++axis) serial.loads[axis*packet::Nodes+secondary]=0;
    } else if (fault==1) {
      serial.trial.back()=DBL_MAX;
      serial.trial[serial.TailOffset()+secondary]=DBL_MAX/2;
    }
    auto actual=serial;const auto accepted=serial.accepted;
    RunPair(serial,actual,true);
    ASSERT_EQ(serial.control.status,NodalStatus::InvalidOutput);
    packet::SameControl(serial.control,actual.control);old::SameForce(serial,actual);
    EXPECT_EQ(serial.failure,actual.failure);packet::SameDoubles(actual.accepted,accepted);
    auto retry=old::Population(129,true,true),reference=retry;
    retry.Begin(2);reference.Begin(2);cin_input_test::Seed(retry);cin_input_test::Seed(reference);
    RunPair(reference,retry,true);
    ASSERT_EQ(retry.control.status,NodalStatus::Ok);
    packet::SameSuccessfulPacket(reference,retry);old::SamePatches(reference,retry);
  }
}
TEST(CinMasterGatherCuda,RejectedInputsLeaveOldWorkspaceAndPatchesUntouched) {
  for (auto fault:{cin_input_test::Fault::NodeBeforeWitness,cin_input_test::Fault::NumericalBeforeWitness,
                  cin_input_test::Fault::LateRow,cin_input_test::Fault::NoRows}) {
    auto serial=old::Population(2,true,true),actual=serial;
    cin_input_test::Inject(serial,fault);cin_input_test::Inject(actual,fault);
    const auto before=actual;
    RunPair(serial,actual,false);
    ASSERT_NE(serial.control.status,NodalStatus::Ok);
    packet::SameControl(serial.control,actual.control);old::SameForce(serial,actual);
    packet::SameDoubles(actual.work,before.work);packet::SameDoubles(actual.accepted,before.accepted);
  }
}
} // namespace tl::fea::cin_gather_test
