// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../constraints/NodalRigidAssemblyBinding.h"
#include "../elements/ShellBatchPlasticityBinding.h"

namespace tl::fea {
struct ShellExecutionLimits {
  std::size_t max_parents = 2048, max_nodes = 2048;
  std::size_t max_host_bytes = 64u << 20;
  static constexpr ShellExecutionLimits Vehicle() noexcept {
    return {524288, 524288, 2ull << 30};
  }
};
struct ShellExecutionParent {
  ShellPlasticityParentInput source;
  ShellSectionLaw law = ShellSectionLaw::Unspecified;
  unsigned material_points = 0;
  // SIZE_MAX for constitutive rows. The original PART may be a merged child;
  // root_index is also the index of its actual PART group in rigid().groups().
  std::size_t part_index = SIZE_MAX, root_index = SIZE_MAX;
};
struct ShellExecutionCounts {
  std::size_t constitutive = 0, rigid_skin = 0, material_points = 0;
};
struct ShellExecutionForecast {
  std::size_t owned_payload_bytes = 0, startup_payload_bytes = 0;
};

// Complete immutable execution declarations joined to the actual physical
// ledger and prepared PART assembly. The catalog supplies explicit roles;
// membership alone never changes a constitutive shell into a rigid skin.
// Keeps all reference geometry, M/J and source/family indices. This host binding
// neither advances material nor admits a participant, surface activity or DOFs.
class ShellExecutionBinding {
 public:
  ShellExecutionBinding() = default;
  ShellExecutionBinding(const ShellExecutionBinding&) noexcept = default;
  ShellExecutionBinding(ShellExecutionBinding&& other) noexcept : impl_(other.impl_) {}
  ShellExecutionBinding& operator=(const ShellExecutionBinding&) = delete;
  static ShellPlasticityBindingReport Preflight(const ShellBatchPlasticityBinding&,
      const NodalCoefficientLedger&, const NodalRigidAssemblyBinding&,
      ShellExecutionLimits, ShellExecutionForecast*) noexcept;
  ShellPlasticityBindingReport Initialize(const ShellBatchPlasticityBinding&,
      const NodalCoefficientLedger&, const NodalRigidAssemblyBinding&,
      ShellExecutionLimits = {}) noexcept;
  bool prepared() const noexcept { return bool(impl_); }
  const ShellBatchPlasticityBinding* catalog() const noexcept;
  const NodalRigidAssemblyBinding* rigid() const noexcept;
  const NodalCoefficientLedger* coefficients() const noexcept;
  const ShellNodeMap* mapping() const noexcept;
  const ShellBatchBinding* shells() const noexcept;
  const NodalNodeDomain* domain() const noexcept;
  tl::util::ConstView<ShellExecutionParent> parents() const noexcept;
  const ShellExecutionParent* parent(std::size_t source_row) const noexcept;
  const ShellExecutionParent* parent(ShellBindingFamily, std::size_t family_index) const noexcept;
  ShellExecutionCounts counts() const noexcept;
  ShellExecutionForecast forecast() const noexcept;
  bool Matches(const ShellBatchPlasticityBinding&, const NodalCoefficientLedger&) const noexcept;
  // Exact shared immutable handle identity; copies retain the same authority.
  bool Matches(const ShellExecutionBinding&) const noexcept;
 private:
  struct Impl;
  std::shared_ptr<const Impl> impl_;
};
} // namespace tl::fea
