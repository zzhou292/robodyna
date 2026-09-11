#include "NativeAnalytic.h"
#include "NativePointInternal.h"
#include <limits>

extern "C" void law44_analytic_point(double,double,double,double,double,double,
    const double*,const double*,const double*,double,double,double*,double*);
namespace tl::qualification::law44 {
bool EvaluateAnalytic(const AnalyticInput& source,double layer,double thickness,AnalyticResult& output) {
  const auto& in=source.point;
  const double native_default_limit=static_cast<double>(1e20f); // CONSTANT_MOD, HM_READ_MAT44 SI branch.
  if(!detail::ValidCommonInput(in)||!in.rate.active||in.plastic_strain||in.yield_stress||in.point_count||
     in.continuation!=CurveContinuation::StrictDomain||
     !std::isfinite(source.initial_yield)||source.initial_yield<=0||source.initial_yield>=native_default_limit||
     !std::isfinite(source.tangent_modulus)||source.tangent_modulus<0||source.tangent_modulus>=in.young||
     !std::isfinite(layer)||layer<=0||!std::isfinite(thickness)||thickness<=0) return false;
  // Native mapping is allowed to overflow only into a failed trial, never a
  // published result. This guard also avoids invalid native power arguments.
  const long double modulus=static_cast<long double>(source.tangent_modulus)*in.young/(in.young-source.tangent_modulus);
  if(!std::isfinite(modulus)||modulus>std::numeric_limits<double>::max()) return false;
  std::array<double,6> base{};
  std::copy(in.accepted_stress.begin(),in.accepted_stress.end(),base.begin()); base[5]=in.accepted_plastic_strain;
  const auto& r=in.rate;
  const std::array<double,5> rate{r.coefficient_per_s,r.exponent,r.total_shell_rate_per_s,
      r.filter_coefficient,r.accepted_filtered_rate_per_s};
  std::array<double,13> values{}; double b=0;
  law44_analytic_point(in.young,in.poisson,in.density,in.transverse_shear_modulus,
      source.initial_yield,source.tangent_modulus,base.data(),in.strain_increment.data(),rate.data(),
      layer,thickness,values.data(),&b);
  for(double x:values) if(!std::isfinite(x)) return false;
  if(!std::isfinite(b)||b<0||(source.tangent_modulus>0&&b==0)||values[11]!=1||values[8]<=0||
      values[5]<in.accepted_plastic_strain) return false;
  const double stress_limit=b==0?native_default_limit:(native_default_limit-source.initial_yield)/b;
  if(values[5]>=native_default_limit||values[5]>=stress_limit) return false;
  AnalyticResult candidate;
  if(!detail::DecodeResponse(in,values,candidate)) return false;
  candidate.reported_thickness_m=values[8]; candidate.native_plastic_hardening=b;
  output=candidate; return true;
}
} // namespace tl::qualification::law44
