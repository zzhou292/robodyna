// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../cin_force_transfers/Fixture.h"
#include "../cin_force_inputs/Serial.h"
#include "../cin_parallel_ordinary/DevicePacket.h"
namespace tl::fea::cin_input_test {
TEST(CinPostNodeInputsCuda, BlockBoundaryWitnessAndRowChecksMatchFrozenCallerThenRetry) {
  for (unsigned fault=0;fault<6;++fault) {
    SCOPED_TRACE(fault);
    auto original=cin_transfer_test::Population(129,true,true);
    auto expected=original;
    if (fault==0) {expected.first_witness[128]=127;expected.activity[128]=2;expected.rows[0].masters[0]=Nodes;}
    if (fault==1) {expected.rows[127].masters[0]=Nodes;expected.activity[1]=0;}
    if (fault==2) {expected.rows[128].masters[3]=Nodes;expected.rows[127].masters[1]=Nodes;}
    if (fault==3) {expected.trial.back()=NAN;expected.first_witness[128]=129;}
    if (fault==4) {expected.work[128]=NAN;expected.trial.back()=NAN;}
    auto actual=expected;
    for (unsigned attempt=0;attempt<2;++attempt) {
      DevicePacket serial(expected,true), parallel(actual,true);
      serial.RunWith(LaunchFrozen);parallel.RunWith(cin_advance::Launch);
      serial.Download(expected);parallel.Download(actual);
      SameControl(actual.control,expected.control);SameForce(actual,expected);
      cin_transfer_test::SamePatches(actual,expected);
      if (attempt || fault==5) SameSuccessfulPacket(actual,expected);
      actual=expected=original;actual.Begin(2);expected.Begin(2);Seed(actual);Seed(expected);
    }
  }
}
}
