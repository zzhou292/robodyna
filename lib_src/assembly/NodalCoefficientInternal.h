// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodalCoefficientLedger.h"
#include "NodalDomainIdentity.h"
#include "../../lib_utils/BoundedArena.h"
#include <cmath>

namespace tl::fea::coefficient_detail {
using S=CoefficientStatus;
using P=CoefficientProducer;
struct Budget {
  util::BoundedArenaLayout arena;
  util::ArenaRegion nodes;
  std::size_t retained=0,startup=0;
  explicit Budget(std::size_t cap):arena(cap) {}
};
CoefficientReport Preflight(NodalCoefficientSources,CoefficientLimits,
                            std::size_t implementation_bytes,Budget&) noexcept;
CoefficientReport Identities(NodalCoefficientSources);
CoefficientReport Shells(const ShellNodeMap&,NodalCoefficientNode*) noexcept;
CoefficientReport Springs(NodalCoefficientSources,NodalCoefficientNode*) noexcept;
CoefficientReport Totals(NodalCoefficientNode*,std::size_t,
                         NodalCoefficientTotals&,NodalCoefficientScope&) noexcept;
inline bool Positive(double value) noexcept {return std::isfinite(value)&&value>0;}
inline bool Nonnegative(double value) noexcept {return std::isfinite(value)&&value>=0;}
inline bool Add(double& sum,double value) noexcept {
  sum+=value;
  return Nonnegative(sum);
}
inline bool AddPair(double& mass,double& inertia,double m,double j) noexcept {
  return Add(mass,m)&&Add(inertia,j);
}
} // namespace tl::fea::coefficient_detail
