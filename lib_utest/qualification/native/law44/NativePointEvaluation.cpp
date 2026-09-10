#include "NativePointInternal.h"
#include <vector>
extern "C" void law44_point_evaluate(int,const double*,double,double,double,double,
    const double*,const double*,const double*,double*);
extern "C" void law44_point_physical(int,const double*,double,double,double,double,
    const double*,const double*,const double*,double,double,double*);
namespace tl::qualification::law44::detail {
bool EvaluateValues(const Input& in, bool physical, double layer, double thickness,
                    std::array<double,13>& output) {
    if (!ValidCommonInput(in)||!in.plastic_strain||!in.yield_stress||in.point_count<2||in.point_count>1024)
        return false;
    for (std::size_t n=0; n<in.point_count; ++n) {
        if (!std::isfinite(in.plastic_strain[n]) || !std::isfinite(in.yield_stress[n]) ||
            in.yield_stress[n]<=0 || (n==0 && in.plastic_strain[n]!=0) ||
            (n && (in.plastic_strain[n]<=in.plastic_strain[n-1] ||
                   in.yield_stress[n]<in.yield_stress[n-1]))) return false;
    }
    if (in.accepted_plastic_strain>in.plastic_strain[in.point_count-1]) return false;
    std::vector<double> curve(2*(in.point_count+1), 0);
    for (std::size_t n=0; n<in.point_count; ++n) {
        curve[2*(n+1)]=in.plastic_strain[n];
        curve[2*(n+1)+1]=in.yield_stress[n];
    }
    std::array<double, 6> base{};
    std::copy(in.accepted_stress.begin(), in.accepted_stress.end(), base.begin());
    base[5]=in.accepted_plastic_strain;
    const auto& rate=in.rate;
    const std::array<double,5> rate_values{rate.coefficient_per_s,rate.exponent,
        rate.total_shell_rate_per_s,rate.filter_coefficient,rate.accepted_filtered_rate_per_s};
    std::array<double, 13> values{};
    if(physical) law44_point_physical(static_cast<int>(in.point_count), curve.data(), in.young,
        in.poisson, in.density, in.transverse_shear_modulus, base.data(),
        in.strain_increment.data(), rate_values.data(), layer, thickness, values.data());
    else law44_point_evaluate(static_cast<int>(in.point_count), curve.data(), in.young,
        in.poisson, in.density, in.transverse_shear_modulus, base.data(),
        in.strain_increment.data(), rate_values.data(), values.data());
    for (double x:values) if (!std::isfinite(x)) return false;
    if (values[11]!=1 || values[5]<in.accepted_plastic_strain ||
        values[5]>in.plastic_strain[in.point_count-1]) return false;
    output=values; return true;
}
} // namespace tl::qualification::law44::detail
