// SPDX-License-Identifier: AGPL-3.0-or-later
// OpenRadioss (C) 2026 Siemens; selected MYREAL8 constant_mod expressions.
// Exact map and arithmetic domain: qualification/t3/FORCE.md.
#pragma once
namespace tl::fea::t3::detail::force_constant {
constexpr double ep10=100000.*100000.,ep20=ep10*ep10;
constexpr double em20=1./ep20,em30=1./(ep20*ep10);
constexpr double third=1./3.,fourth=1./4.,one_over_9=1./9.;
constexpr double one_over_12=1./12.,four_over_3=4./3.,five_over_6=5./6.;
constexpr double zep01=1./100.,five_em3=5./1000.,four_em3=4./1000.;
constexpr double onep4=1.+4./10.,onep41=onep4+zep01,onep414=onep41+four_em3;
constexpr double viscosity=zep01+five_em3;
// The shared LAW1 helper uses 1e20 in its sound-speed floor. Integer powers
// through 1e20 have exact binary64 representations; keep equivalence checked.
static_assert(ep20==1e20&&em20==1./1e20,"Native/shared density floor identity");
static_assert(em30==1./(1e20*1e10),"Native reported-thickness floor identity");
} // namespace tl::fea::t3::detail::force_constant
