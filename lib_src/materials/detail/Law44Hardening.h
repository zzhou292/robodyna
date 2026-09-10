// SPDX-License-Identifier: AGPL-3.0-or-later
// convertmats.cxx / SIGEPS44C analytic branch; OpenRadioss (C) 2026 Siemens.
#pragma once
namespace tl::material {
namespace tabulated_shell_detail {
// Pinned CONSTANT_MOD INFINITY=1E20 is a default-real literal promoted to my_real.
// These source-SI default caps are not a shell failure/deletion implementation.
TL_TABULATED_SHELL_HD inline double AnalyticDefaultLimit() noexcept { return static_cast<double>(1e20f); }
TL_TABULATED_SHELL_HD inline bool EmptyCurve(TabulatedShellPlasticityCurve c) noexcept {
  return !c.plastic_strain&&!c.yield_stress_pa&&c.count==0;
}
TL_TABULATED_SHELL_HD inline bool LinearModulus(double young,Law44LinearHardening in,double& b) noexcept {
  if(!tl::math::Finite(in.initial_yield_pa)||in.initial_yield_pa<=0||in.initial_yield_pa>=AnalyticDefaultLimit()||
     !tl::math::Finite(in.tangent_modulus_pa)||in.tangent_modulus_pa<0||in.tangent_modulus_pa>=young)
    return false;
  // Exact converter expression; do not rewrite as E/(E/ETAN-1).
  const double candidate=in.tangent_modulus_pa*young/(young-in.tangent_modulus_pa);
  if(!tl::math::Finite(candidate)||candidate<0||
      (in.tangent_modulus_pa>0&&candidate==0)) return false;
  b=candidate; return true;
}
TL_TABULATED_SHELL_HD inline TabulatedShellPlasticityStatus ValidHardening(
    const TabulatedShellPlasticityParameters& p) noexcept {
  using Status=TabulatedShellPlasticityStatus;
  if(p.hardening==ShellPlasticityHardeningKind::Tabulated) {
    if(p.linear.initial_yield_pa!=0||p.linear.tangent_modulus_pa!=0||p.plastic_hardening_pa!=0)
      return Status::InvalidParameters;
    return CurveShape(p.curve)?Status::Ok:Status::InvalidCurve;
  }
  double b=0;
  if(p.hardening!=ShellPlasticityHardeningKind::LinearLaw44||!EmptyCurve(p.curve)||!p.rate.enabled||
     !LinearModulus(p.young_pa,p.linear,b)||b!=p.plastic_hardening_pa) return Status::InvalidParameters;
  return Status::Ok;
}
TL_TABULATED_SHELL_HD inline bool HardeningDomain(const TabulatedShellPlasticityParameters& p,double pla) noexcept {
  if(p.hardening==ShellPlasticityHardeningKind::Tabulated) return pla<=p.curve.plastic_strain[p.curve.count-1];
  const double limit=AnalyticDefaultLimit();
  const double stress_cap=p.plastic_hardening_pa==0?limit:(limit-p.linear.initial_yield_pa)/p.plastic_hardening_pa;
  return pla<limit&&pla<stress_cap;
}
TL_TABULATED_SHELL_HD inline bool HardeningValue(const TabulatedShellPlasticityParameters& p,double pla,
    double& yield,double& slope) noexcept {
  if(p.hardening==ShellPlasticityHardeningKind::Tabulated) return CurveValue(p.curve,pla,yield,slope);
  yield=pla>0?p.linear.initial_yield_pa+p.plastic_hardening_pa*pla:p.linear.initial_yield_pa;
  slope=p.plastic_hardening_pa;
  return tl::math::Finite(yield)&&yield>0;
}
} // namespace tabulated_shell_detail

TL_TABULATED_SHELL_HD inline TabulatedShellPlasticityStatus PrepareLinearLaw44ShellPlasticity(
    double young,double nu,double rho,Law44LinearHardening linear,TabulatedShellPlasticityRate rate,
    TabulatedShellPlasticityParameters& output) noexcept {
  using Status=TabulatedShellPlasticityStatus;
  if(!tabulated_shell_detail::ValidElasticInput(young,nu,rho)||!rate.enabled) return Status::InvalidParameters;
  TabulatedShellPlasticityParameters p;
  if(!tabulated_shell_detail::LinearModulus(young,linear,p.plastic_hardening_pa)||
     !tabulated_shell_detail::PrepareRate(p,rate)||!tabulated_shell_detail::PrepareElastic(young,nu,rho,p))
    return Status::InvalidParameters;
  p.hardening=ShellPlasticityHardeningKind::LinearLaw44; p.linear=linear;
  output=p; return Status::Ok;
}
} // namespace tl::material
