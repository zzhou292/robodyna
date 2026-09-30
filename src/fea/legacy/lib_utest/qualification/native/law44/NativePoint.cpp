#include "NativePointInternal.h"
#include <algorithm>
#include <cmath>
#include <limits>

extern "C" void law44_point_section(double*, double*, double*);
extern "C" double law44_point_filter(double, double);
extern "C" double law44_point_shell_rate(const double*, double, double);

namespace tl::qualification::law44 {
double NativeFilterCoefficient(double cutoff_per_s,double dt) {
    if(!std::isfinite(cutoff_per_s)||cutoff_per_s<=0||!std::isfinite(dt)||dt<=0)
        return std::numeric_limits<double>::quiet_NaN();
    return law44_point_filter(cutoff_per_s,dt);
}
double NativeShellRate(const std::array<double, 8>& increment,
                       double accepted_thickness, double dt) {
    if (!std::isfinite(accepted_thickness) || accepted_thickness <= 0 ||
        !std::isfinite(dt) || dt <= 0)
        return std::numeric_limits<double>::quiet_NaN();
    for (double x : increment)
        if (!std::isfinite(x)) return std::numeric_limits<double>::quiet_NaN();
    return law44_point_shell_rate(increment.data(), accepted_thickness, dt);
}
SectionRule NativeSectionRule() {
    SectionRule rule;
    law44_point_section(rule.position.data(), rule.membrane_weight.data(),
                        rule.moment_weight.data());
    return rule;
}
bool Evaluate(const Input& in,Result& output) {
    std::array<double,13> values{};
    if(!detail::EvaluateValues(in,false,1.,0.,values)) return false;
    Result candidate;
    if(!detail::DecodeResponse(in,values,candidate)) return false;
    candidate.total_thickness_strain=values[8];
    output=candidate; return true;
}
} // namespace tl::qualification::law44
