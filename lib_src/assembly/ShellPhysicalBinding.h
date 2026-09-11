// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodalCoefficientLedger.h"
#include "../elements/ShellFormulationScope.h"

namespace tl::fea {
struct ShellPhysicalBindingLimits {
  std::size_t max_nodes = 2048, max_parents = 2048;
  std::size_t max_host_bytes = 64u << 20;
  static constexpr ShellPhysicalBindingLimits Vehicle() noexcept {
    return {524288,524288,2ull << 30};
  }
};

// Complete shell material/failure scope joined to an already prepared physical
// ledger. The ledger owns the exact shell-to-domain map; the failure handle owns
// its catalog. Retaining these two handles adds no second node/material arena.
// Other families still require their own participants and a common publication.
// This object admits no DOFs, constrained inverse coefficients, time or forces.
class ShellPhysicalBinding {
 public:
  ShellPhysicalBinding() = default;
  ShellPhysicalBinding(const ShellPhysicalBinding&) noexcept = default;
  ShellPhysicalBinding(ShellPhysicalBinding&& other) noexcept : impl_(other.impl_) {}
  ShellPhysicalBinding& operator=(const ShellPhysicalBinding&) = delete;
  NodalDomainReport Initialize(const ShellFormulationScope&, const NodalCoefficientLedger&,
                              ShellPhysicalBindingLimits = {}) noexcept;
  bool prepared() const noexcept { return bool(impl_); }
  const NodalCoefficientLedger* coefficients() const noexcept;
  const ShellBatchFailureBinding* failure() const noexcept;
  const ShellBatchPlasticityBinding* catalog() const noexcept;
  const ShellNodeMap* mapping() const noexcept;
  const ShellBatchBinding* shells() const noexcept;
  const NodalNodeDomain* domain() const noexcept;
  bool Matches(const ShellPhysicalBinding&) const noexcept;
  std::size_t owned_payload_bytes() const noexcept;
 private:
  struct Impl;
  std::shared_ptr<const Impl> impl_;
};
} // namespace tl::fea
