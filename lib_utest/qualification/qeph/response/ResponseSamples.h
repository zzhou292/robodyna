#pragma once
#include "ResponseData.h"

namespace tl::qualification::qeph::response {
// Output-only endpoint reconstruction: add h/2 times the cached endpoint total
// RHS to carried midpoint rates. It does not evaluate force or advance history.
// Initial physical velocities are retained without an extrapolated half kick.
bool Observe(const Model&,double h,std::uint64_t epoch,const State&,const Results&,
             long double external_work,Sample&,Limits&,std::string& error);
void AccumulateLimits(Limits&,const Limits&) noexcept;
bool InsideLimits(const Limits&) noexcept;
Extremum ResponseMaximum(const std::vector<Field>&,const Sample& initial,const Sample&);
} // namespace tl::qualification::qeph::response
