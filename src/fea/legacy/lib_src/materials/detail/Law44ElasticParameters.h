// SPDX-License-Identifier: AGPL-3.0-or-later
// HM_READ_MAT44 coefficients adapted from OpenRadioss (C) 2026 Siemens.
#pragma once
namespace tl::material::tabulated_shell_detail {
TL_TABULATED_SHELL_HD inline bool ValidElasticInput(double young,double nu,double rho) noexcept {
  return tl::math::Finite(young)&&young>0&&tl::math::Finite(nu)&&nu>=0&&nu<.5&&
      tl::math::Finite(rho)&&rho>0;
}
// The caller stages p privately. Operation order is the existing table path.
TL_TABULATED_SHELL_HD inline bool PrepareElastic(double young,double nu,double rho,
    TabulatedShellPlasticityParameters& p) noexcept {
  p.young_pa=young; p.poisson_ratio=nu; p.density_kg_m3=rho;
  p.shear_modulus=young/2./(1.+nu);
  p.a11=young/(1.-nu*nu); p.a12=p.a11*nu;
  p.three_g=3.*p.shear_modulus; p.sound_speed=::sqrt(p.a11/rho);
  return tl::math::Finite(p.shear_modulus)&&p.shear_modulus>0&&
      tl::math::Finite(p.a11)&&p.a11>0&&tl::math::Finite(p.a12)&&
      tl::math::Finite(p.three_g)&&p.three_g>0&&tl::math::Finite(p.sound_speed)&&p.sound_speed>0;
}
} // namespace tl::material::tabulated_shell_detail
