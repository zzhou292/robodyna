#pragma once
#include "NativePoint.h"

namespace tl::qualification::law44 {
struct PhysicalThicknessResult {
    std::array<double,5> stress{};
    double plastic_strain=0,plastic_increment=0,tangent_ratio=0;
    double reported_thickness_m=0,yield_before=0,sound_speed=0;
    double equivalent_stress=0,plastic_work_density=0,filtered_rate_per_s=0;
};
// Complete SIGEPS44C receives actual THKLY and running THKN. Its elastic and
// plastic thickness additions execute separately, in native source order.
// This is a proposed value update, not a section/integrator/clock. Failed input,
// native output or vanished thickness preserves the complete caller output.
bool EvaluatePhysicalThickness(const Input&,double layer_thickness_m,
    double running_thickness_m,PhysicalThicknessResult&);
} // namespace tl::qualification::law44
