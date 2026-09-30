#pragma once
#include "GroupStepTestSupport.h"
namespace rigid_step_test {
// Packet setup is authored here; native arithmetic fragments are retained and
// manifest-verified. This is not the complete engine or its startup scheduler.
Trial NativePacket(const Input&);
Trial NativeTwoPacket(const Input&,double source_length_to_m);
tl::math::Matrix3 NativeFrame(const tl::math::Matrix3&,Vec3 body_omega,double dt);
} // namespace rigid_step_test
