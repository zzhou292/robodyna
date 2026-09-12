// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodalCoefficientTypes.h"
#include "ShellNodeMap.h"
#include "Type13NodeContributions.h"
#include "ElementMassContributions.h"
#include "SolidNodeContributions.h"
#include "Beam18NodeContributions.h"
#include "../elements/type25/Type25Model.h"

namespace tl::fea {
struct NodalCoefficientSources {
  const ShellNodeMap* shells=nullptr;
  const type25::Model* type25=nullptr;
  const Type13NodeContributions* type13=nullptr;
};
struct NodalCoefficientSourcesWithElementMass {
  NodalCoefficientSources structural;
  const ElementMassContributions* element_mass=nullptr;
};
struct NodalCoefficientSourcesWithSolids {
  NodalCoefficientSources structural;
  const ElementMassContributions* element_mass=nullptr;
  const SolidNodeContributions* solids=nullptr;
};
struct NodalCoefficientSourcesWithBeams {
  NodalCoefficientSourcesWithSolids physical;
  const Beam18NodeContributions* beam18=nullptr;
};
// Closed immutable additive startup values. Complete admitted producers are
// retained; uncovered nodes stay explicit zero rows. Neither covered nodes nor
// successful initialization prove full-source coverage, DOFs or owner admission.
// No coefficient overrides, inverse values or native global reduction claim.
class NodalCoefficientLedger {
 public:
  NodalCoefficientLedger()=default;
  NodalCoefficientLedger(const NodalCoefficientLedger&) noexcept=default;
  NodalCoefficientLedger(NodalCoefficientLedger&& other) noexcept
      :NodalCoefficientLedger(static_cast<const NodalCoefficientLedger&>(other)) {}
  NodalCoefficientLedger& operator=(const NodalCoefficientLedger&)=delete;
  CoefficientReport Initialize(NodalCoefficientSources,CoefficientLimits={}) noexcept;
  CoefficientReport InitializeWithElementMass(NodalCoefficientSourcesWithElementMass,
                                             CoefficientLimits={}) noexcept;
  CoefficientReport InitializeWithSolids(NodalCoefficientSourcesWithSolids,
                                        CoefficientLimits={}) noexcept;
  // Explicit V4 order: the legacy solid families followed by LAW44 and LAW90.
  // Requires an extended immutable snapshot. This adds mass only; admitting
  // mechanics, constraints and force publication remains the owner's task.
  CoefficientReport InitializeWithExtendedSolids(NodalCoefficientSourcesWithSolids,
                                                CoefficientLimits={}) noexcept;
  // Explicit V5 appends native beam M/J after all admitted solid families.
  // A beam snapshot is required; existing V1-V4 entry points remain closed.
  CoefficientReport InitializeWithBeams(NodalCoefficientSourcesWithBeams,CoefficientLimits={}) noexcept;
  bool prepared() const noexcept {return bool(impl_);}
  const ShellNodeMap* shells() const noexcept;
  const type25::Model* type25() const noexcept;
  const Type13NodeContributions* type13() const noexcept;
  const ElementMassContributions* element_mass() const noexcept;
  const SolidNodeContributions* solids() const noexcept;
  const Beam18NodeContributions* beam18() const noexcept;
  const NodalNodeDomain* domain() const noexcept;
  tl::util::ConstView<NodalCoefficientNode> nodes() const noexcept;
  const NodalCoefficientTotals& totals() const noexcept;
  const NodalCoefficientScope& scope() const noexcept;
  bool Matches(NodalCoefficientSources) const noexcept;
  bool MatchesWithElementMass(NodalCoefficientSourcesWithElementMass) const noexcept;
  bool MatchesWithSolids(NodalCoefficientSourcesWithSolids) const noexcept;
  bool MatchesWithExtendedSolids(NodalCoefficientSourcesWithSolids) const noexcept;
  bool MatchesWithBeams(NodalCoefficientSourcesWithBeams) const noexcept;
  bool Matches(const NodalCoefficientLedger&) const noexcept;
  std::size_t owned_payload_bytes() const noexcept;
  std::size_t startup_payload_bytes() const noexcept;
  CoefficientOrder order() const noexcept;
 private:
  CoefficientReport InitializeImpl(NodalCoefficientSourcesWithSolids,
                                  CoefficientOrder,CoefficientLimits,const Beam18NodeContributions* =nullptr) noexcept;
  bool MatchesSolids(NodalCoefficientSourcesWithSolids,CoefficientOrder) const noexcept;
  struct Impl;
  std::shared_ptr<const Impl> impl_;
};
} // namespace tl::fea
