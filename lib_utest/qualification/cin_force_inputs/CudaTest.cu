// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "Serial.h"
#include "../cin_parallel_ordinary/DevicePacket.h"
#include "../cin_parallel_ordinary/Fault.h"

namespace tl::fea::cin_input_test {
namespace {
void RunPacket(Packet& packet, bool staged) {
  DevicePacket device(packet, true);
  device.RunWith(staged ? cin_advance::Launch : LaunchFrozen);
  device.Download(packet);
}
}
TEST(CinForceInputsCuda, AllFieldsMatchCompleteFrozenCallerThroughBothTemporalPhases) {
  for (bool groups : {false, true}) {
    for (bool capture : {false, true}) {
      for (bool screen : {false, true}) {
        Packet original(groups, capture);
        if (screen) original.structural = {NodalCinStructuralProfile::NativeOrdinaryRigidTrace, .8};
        Packet staged = original;
        const auto loads = original.loads;
        for (unsigned step = 0; step < 3; ++step) {
          SCOPED_TRACE(step);
          original.Begin(step+1);
          staged.Begin(step+1);
          Seed(original);
          Seed(staged);
          original.loads = staged.loads = loads;
          RunPacket(original, false);
          RunPacket(staged, true);
          SameSuccessfulPacket(staged, original);
          EXPECT_EQ(staged.input_failure, cin_advance::NoFailure);
          EXPECT_EQ(staged.failure, cin_advance::NoFailure);
          original.Accept();
          staged.Accept();
        }
      }
    }
  }
}
TEST(CinForceInputsCuda, EveryInputFailurePreservesControlScratchAndOrdinaryKeyThenRetries) {
  using F = Fault;
  for (auto fault : {F::EarlyMass, F::BoundaryInertia, F::LastPosition,
      F::LastCouple, F::LateStiffness, F::NodeBeforeWitness, F::NodeBeforeNumerical,
      F::NumericalBeforeWitness, F::WitnessBeforeRow, F::WitnessAlias,
      F::LateRow, F::MissingActivity, F::NoRows, F::StaleBeforeNode, F::LimitBeforeNode}) {
    SCOPED_TRACE(int(fault));
    Packet initial(true, true);
    Seed(initial);
    Packet bad = initial;
    Inject(bad, fault);
    Packet expected = bad, staged = bad;
    RunPacket(expected, false);
    RunPacket(staged, true);
    ASSERT_NE(expected.control.status, NodalStatus::Ok);
    SameControl(staged.control, expected.control);
    UnchangedBeforeTransfer(staged, bad);
    UnchangedBeforeTransfer(expected, bad);
    staged = expected = initial;
    staged.Begin(2);
    expected.Begin(2);
    Seed(staged);
    Seed(expected);
    RunPacket(expected, false);
    RunPacket(staged, true);
    SameSuccessfulPacket(staged, expected);
  }
}
TEST(CinForceInputsCuda, TransferScreenAndMotionFailuresKeepOriginalPhasePriority) {
  using F = cin_parallel_test::Fault;
  for (auto fault : {F::AngleBeforeInverse, F::InverseBeforeAngle, F::PendingBeforeMotion,
      F::GeometryBeforeMotion, F::ScreenBeforeMotion, F::OrdinaryBeforeGroup,
      F::GroupZeroOrientationBeforeGroupOnePrimary}) {
    SCOPED_TRACE(int(fault));
    Packet initial(true, true);
    Seed(initial);
    Packet expected = initial;
    cin_parallel_test::Inject(expected, fault);
    Packet staged = expected;
    // Some faults deliberately edit accepted geometry/orientation. Preserve
    // that actual input; the unaffected pre-injection fixture is for retry.
    const auto accepted = staged.accepted;
    RunPacket(expected, false);
    RunPacket(staged, true);
    ASSERT_NE(expected.control.status, NodalStatus::Ok);
    SameControl(staged.control, expected.control);
    SameDoubles(staged.accepted, accepted);
    if (fault == F::PendingBeforeMotion || fault == F::GeometryBeforeMotion || fault == F::ScreenBeforeMotion) {
      EXPECT_EQ(staged.failure, initial.failure);
      SameDoubles(staged.work, expected.work);
      SameDoubles(staged.loads, expected.loads);
      SameDoubles(staged.trial, expected.trial);
      SameDoubles(staged.capture, initial.capture);
    }
    staged = expected = initial;
    staged.Begin(2);
    expected.Begin(2);
    RunPacket(expected, false);
    RunPacket(staged, true);
    SameSuccessfulPacket(staged, expected);
  }
}
} // namespace tl::fea::cin_input_test
