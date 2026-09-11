#pragma once
#include "NativePoint.h"
#include <algorithm>
#include <cmath>

namespace tl::qualification::law44::detail {
bool EvaluateValues(const Input&,bool physical,double layer,double thickness,
                    std::array<double,13>&);
inline bool ValidCommonInput(const Input& in) {
    if(!std::isfinite(in.young)||in.young<=0||!std::isfinite(in.poisson)||in.poisson<0||in.poisson>=.5||
       !std::isfinite(in.density)||in.density<=0||!std::isfinite(in.transverse_shear_modulus)||
       in.transverse_shear_modulus<=0||!std::isfinite(in.accepted_plastic_strain)||in.accepted_plastic_strain<0) return false;
    for(double x:in.accepted_stress) if(!std::isfinite(x)) return false;
    for(double x:in.strain_increment) if(!std::isfinite(x)) return false;
    const auto& r=in.rate;
    for(double x:{r.coefficient_per_s,r.exponent,r.total_shell_rate_per_s,r.filter_coefficient,r.accepted_filtered_rate_per_s})
        if(!std::isfinite(x)||x<0) return false;
    if(r.filter_coefficient>1) return false;
    return r.active?(r.coefficient_per_s>0&&r.exponent>0):
        (r.coefficient_per_s==0&&r.exponent==0&&r.total_shell_rate_per_s==0&&r.accepted_filtered_rate_per_s==0);
}
inline double Equivalent(const std::array<double,5>& s) {
    const long double x=s[0],y=s[1],xy=s[2];
    return static_cast<double>(std::sqrt(x*x+y*y-x*y+3*xy*xy));
}
// Shared native response decoding; the two public operations assign their
// distinct thickness observable separately, without changing point arithmetic.
template<class Response> bool DecodeResponse(const Input& in,
    const std::array<double,13>& values,Response& candidate) {
    std::copy_n(values.begin(),5,candidate.stress.begin());
    candidate.plastic_strain=values[5]; candidate.plastic_increment=values[6];
    candidate.tangent_ratio=values[7]; candidate.yield_before=values[9];
    candidate.sound_speed=values[10]; candidate.filtered_rate_per_s=values[12];
    if(candidate.filtered_rate_per_s<0) return false;
    candidate.equivalent_stress=Equivalent(candidate.stress);
    candidate.plastic_work_density=static_cast<double>(.5L*(Equivalent(in.accepted_stress)+
        candidate.equivalent_stress)*(candidate.plastic_strain-in.accepted_plastic_strain));
    return std::isfinite(candidate.equivalent_stress)&&std::isfinite(candidate.plastic_work_density);
}
} // namespace tl::qualification::law44::detail
