#include "NativePoint.h"
#include <algorithm>
#include <cmath>
#include <vector>

extern "C" void law44_point_evaluate(int, const double*, double, double, double,
    double, const double*, const double*, double*);
extern "C" void law44_point_section(double*, double*, double*);

namespace tl::qualification::law44 {
namespace {
double Equivalent(const std::array<double, 5>& s) {
    const long double x=s[0], y=s[1], xy=s[2];
    return static_cast<double>(std::sqrt(x*x+y*y-x*y+3*xy*xy));
}
}
SectionRule NativeSectionRule() {
    SectionRule rule;
    law44_point_section(rule.position.data(), rule.membrane_weight.data(),
                        rule.moment_weight.data());
    return rule;
}
bool Evaluate(const Input& in, Result& output) {
    if (!std::isfinite(in.young) || in.young<=0 || !std::isfinite(in.poisson) ||
        in.poisson<0 || in.poisson>=.5 || !std::isfinite(in.density) || in.density<=0 ||
        !std::isfinite(in.transverse_shear_modulus) || in.transverse_shear_modulus<=0 ||
        !in.plastic_strain || !in.yield_stress || in.point_count<2 || in.point_count>1024 ||
        !std::isfinite(in.accepted_plastic_strain) || in.accepted_plastic_strain<0) return false;
    for (std::size_t n=0; n<in.point_count; ++n) {
        if (!std::isfinite(in.plastic_strain[n]) || !std::isfinite(in.yield_stress[n]) ||
            in.yield_stress[n]<=0 || (n==0 && in.plastic_strain[n]!=0) ||
            (n && (in.plastic_strain[n]<=in.plastic_strain[n-1] ||
                   in.yield_stress[n]<in.yield_stress[n-1]))) return false;
    }
    if (in.accepted_plastic_strain>in.plastic_strain[in.point_count-1]) return false;
    for (double x:in.accepted_stress) if (!std::isfinite(x)) return false;
    for (double x:in.strain_increment) if (!std::isfinite(x)) return false;
    std::vector<double> curve(2*(in.point_count+1), 0);
    for (std::size_t n=0; n<in.point_count; ++n) {
        curve[2*(n+1)]=in.plastic_strain[n];
        curve[2*(n+1)+1]=in.yield_stress[n];
    }
    std::array<double, 6> base{};
    std::copy(in.accepted_stress.begin(), in.accepted_stress.end(), base.begin());
    base[5]=in.accepted_plastic_strain;
    std::array<double, 12> values{};
    law44_point_evaluate(static_cast<int>(in.point_count), curve.data(), in.young,
        in.poisson, in.density, in.transverse_shear_modulus, base.data(),
        in.strain_increment.data(), values.data());
    for (double x:values) if (!std::isfinite(x)) return false;
    if (values[11]!=1 || values[5]<in.accepted_plastic_strain ||
        values[5]>in.plastic_strain[in.point_count-1]) return false;
    Result candidate;
    std::copy_n(values.begin(), 5, candidate.stress.begin());
    candidate.plastic_strain=values[5]; candidate.plastic_increment=values[6];
    candidate.tangent_ratio=values[7]; candidate.total_thickness_strain=values[8];
    candidate.yield_before=values[9]; candidate.sound_speed=values[10];
    candidate.equivalent_stress=Equivalent(candidate.stress);
    candidate.plastic_work_density=static_cast<double>(.5L*(Equivalent(in.accepted_stress)+
        candidate.equivalent_stress)*candidate.plastic_increment);
    if (!std::isfinite(candidate.equivalent_stress) ||
        !std::isfinite(candidate.plastic_work_density)) return false;
    output=candidate;
    return true;
}
}  // namespace tl::qualification::law44
