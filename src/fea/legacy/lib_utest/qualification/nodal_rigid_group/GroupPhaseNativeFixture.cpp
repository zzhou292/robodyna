#include "GroupPhaseNativeFixture.h"
extern "C" {
void nodal_rigid_native_start(double*,double*,double*);
void nodal_rigid_native_schedule(double*,double*,double*,const double*);
}
namespace rigid_step_test {
NativeSchedule::NativeSchedule() { nodal_rigid_native_start(&previous_,&drift_,&time_); }
rigid::StepDurations NativeSchedule::Next(double dt) {
  nodal_rigid_native_schedule(&previous_,&kick_,&drift_,&dt); return {previous_,kick_,drift_};
}
void CarryNative(const Trial& result,Input& input) {
  input.body.previous_frame=result.primary.force_frame; input.body.center=result.primary.center;
  input.body.velocity=result.primary.velocity; input.body.omega=result.primary.omega;
  for(unsigned i=0;i<Count;++i) {
    input.member[i].position=result.member[i].position; input.member[i].velocity=result.member[i].velocity;
    input.member[i].omega=result.member[i].omega;
  }
}
} // namespace rigid_step_test
