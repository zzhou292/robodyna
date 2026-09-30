#pragma once
#include "GroupStepNativeFixture.h"
namespace rigid_step_test {
// Source-derived fresh DT1/DT2 and kick durations, with an explicitly authored
// fixed-step selector. The force/frame/member math remains NativePacket.
class NativeSchedule {
 public:
  NativeSchedule();
  rigid::StepDurations Next(double prescribed_dt);
 private:
  double previous_=0,kick_=0,drift_=0,time_=0;
};
void CarryNative(const Trial&,Input&);
} // namespace rigid_step_test
