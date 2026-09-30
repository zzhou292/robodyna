// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
namespace tl::material::law42 {
TL_LAW42_HD inline Status Prepare(double mu_pa,double nu,double rho,
                                  double cutoff_pa,Parameters& output) noexcept {
  if (!tl::math::Finite(mu_pa)||mu_pa<=0||!tl::math::Finite(nu)||nu<=-1||nu>=0.5||
      !tl::math::Finite(rho)||rho<=0||!tl::math::Finite(cutoff_pa)||cutoff_pa<=0)
    return Status::InvalidParameters;
  // HM_READ_MAT42: GS=sum(Mu*alpha); retain its original bulk expression.
  const double gs=mu_pa*2;
  const double denominator=3*(1-2*nu);
  const double bulk=gs*(1+nu)/(denominator>1e-20?denominator:1e-20);
  if (!tl::math::Finite(gs)||!tl::math::Finite(bulk)||bulk<=0)
    return Status::InvalidParameters;
  output={mu_pa,nu,bulk,rho,cutoff_pa};
  return Status::Ok;
}
} // namespace tl::material::law42
