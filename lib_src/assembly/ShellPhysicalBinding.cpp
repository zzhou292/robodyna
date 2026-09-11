// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ShellPhysicalBinding.h"
#include "../../lib_utils/BoundedArena.h"
#include <new>

namespace tl::fea {
struct ShellPhysicalBinding::Impl {
  Impl(const NodalCoefficientLedger& c, const ShellBatchFailureBinding& f)
      : coefficients(c), failure(f) {}
  NodalCoefficientLedger coefficients;
  ShellBatchFailureBinding failure;
  std::size_t bytes = 0;
};
NodalDomainReport ShellPhysicalBinding::Initialize(const ShellFormulationScope& scope,
    const NodalCoefficientLedger& coefficients, ShellPhysicalBindingLimits limits) noexcept {
  using S = NodalDomainStatus;
  if (impl_) return {S::AlreadyInitialized,"Physical shell binding is immutable"};
  const auto hard = ShellPhysicalBindingLimits::Vehicle();
  if (!limits.max_nodes || limits.max_nodes > hard.max_nodes ||
      !limits.max_parents || limits.max_parents > hard.max_parents ||
      !limits.max_host_bytes || limits.max_host_bytes > hard.max_host_bytes)
    return {S::ResourceLimit,"Physical shell binding limits exceed scope"};
  const auto checked = ValidateShellFormulationScope(scope);
  if (checked.status != ShellPlasticityBindingStatus::Success)
    return {S::InvalidInput,checked.message};
  if (scope.mass || !coefficients.prepared() || !coefficients.shells() ||
      !coefficients.domain() || !coefficients.shells()->Matches(*scope.binding,*coefficients.domain()))
    return {S::InvalidInput,"One exact physical ledger must replace shell-local combined mass"};
  if (coefficients.domain()->node_count() > limits.max_nodes ||
      scope.catalog->parent_count() > limits.max_parents)
    return {S::ResourceLimit,"Physical shell counts exceed scope"};
  if (coefficients.scope().uncovered_nodes)
    return {S::MissingSource,"Every physical node requires an explicit coefficient producer"};
  // Each producer reports its complete retained payload. Shared inventory can
  // appear in both reports; conservatively charge both without pointer-based
  // subtraction of unknown backing. No source data is copied here.
  const auto ledger_bytes = coefficients.owned_payload_bytes();
  const auto failure_bytes = scope.failure->host_bytes();
  if (ledger_bytes < sizeof(NodalCoefficientLedger) ||
      failure_bytes < sizeof(ShellBatchFailureBinding))
    return {S::ResourceLimit,"Invalid retained physical producer accounting"};
  util::BoundedArenaLayout budget(limits.max_host_bytes);
  util::ArenaRegion ignored;
  if (!budget.Append<unsigned char>(sizeof(*this)+sizeof(Impl)+64,ignored) ||
      !budget.Append<unsigned char>(ledger_bytes-sizeof(NodalCoefficientLedger),ignored) ||
      !budget.Append<unsigned char>(failure_bytes-sizeof(ShellBatchFailureBinding),ignored))
    return {S::ResourceLimit,"Physical shell binding and retained sources exceed byte cap"};
  try {
    auto next = std::make_shared<Impl>(coefficients,*scope.failure);
    next->bytes = budget.bytes();
    impl_ = std::move(next);
    return {};
  } catch (const std::bad_alloc&) {
    return {S::ResourceLimit,"Physical shell binding allocation failed"};
  }
}
const NodalCoefficientLedger* ShellPhysicalBinding::coefficients() const noexcept {
  return impl_ ? &impl_->coefficients : nullptr;
}
const ShellBatchFailureBinding* ShellPhysicalBinding::failure() const noexcept {
  return impl_ ? &impl_->failure : nullptr;
}
const ShellBatchPlasticityBinding* ShellPhysicalBinding::catalog() const noexcept {
  return impl_ ? impl_->failure.catalog() : nullptr;
}
const ShellNodeMap* ShellPhysicalBinding::mapping() const noexcept {
  return impl_ ? impl_->coefficients.shells() : nullptr;
}
const ShellBatchBinding* ShellPhysicalBinding::shells() const noexcept {
  return impl_ ? mapping()->shells() : nullptr;
}
const NodalNodeDomain* ShellPhysicalBinding::domain() const noexcept {
  return impl_ ? impl_->coefficients.domain() : nullptr;
}
bool ShellPhysicalBinding::Matches(const ShellPhysicalBinding& other) const noexcept {
  return impl_ && other.impl_ && (impl_ == other.impl_ ||
      (impl_->coefficients.Matches(other.impl_->coefficients) &&
       impl_->failure.SameScope(other.impl_->failure)));
}
std::size_t ShellPhysicalBinding::owned_payload_bytes() const noexcept {
  return impl_ ? impl_->bytes : sizeof(*this);
}
} // namespace tl::fea
