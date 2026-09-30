// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ShellPhysicalBinding.h"
#include "../../lib_utils/BoundedArena.h"
#include <new>
#include <optional>

namespace tl::fea {
struct ShellPhysicalBinding::Impl {
  Impl(const NodalCoefficientLedger& c, const ShellBatchFailureBinding& f)
      : coefficients(c), failure(f) {}
  NodalCoefficientLedger coefficients;
  ShellBatchFailureBinding failure;
  std::optional<ShellExecutionBinding> execution;
  std::size_t bytes = 0;
};
NodalDomainReport ShellPhysicalBinding::Initialize(const ShellFormulationScope& scope,
    const NodalCoefficientLedger& coefficients, ShellPhysicalBindingLimits limits) noexcept {
  return InitializeImpl(scope, coefficients, limits, nullptr);
}
NodalDomainReport ShellPhysicalBinding::InitializeExecution(const ShellFormulationScope& scope,
    const NodalCoefficientLedger& coefficients, const ShellExecutionBinding& execution,
    ShellPhysicalBindingLimits limits) noexcept {
  return InitializeImpl(scope, coefficients, limits, &execution);
}
NodalDomainReport ShellPhysicalBinding::InitializeImpl(const ShellFormulationScope& scope,
    const NodalCoefficientLedger& coefficients, ShellPhysicalBindingLimits limits,
    const ShellExecutionBinding* execution) noexcept {
  using S = NodalDomainStatus;
  if (impl_) return {S::AlreadyInitialized,"Physical shell binding is immutable"};
  const auto hard = ShellPhysicalBindingLimits::Vehicle();
  if (!limits.max_nodes || limits.max_nodes > hard.max_nodes ||
      !limits.max_parents || limits.max_parents > hard.max_parents ||
      !limits.max_host_bytes || limits.max_host_bytes > hard.max_host_bytes)
    return {S::ResourceLimit,"Physical shell binding limits exceed scope"};
  const auto checked = execution ? ValidateShellExecutionScope(scope) : ValidateShellFormulationScope(scope);
  if (checked.status != ShellPlasticityBindingStatus::Success)
    return {S::InvalidInput,checked.message};
  if (execution && !execution->Matches(*scope.catalog, coefficients))
    return {S::InvalidInput,"Physical shell execution differs from the exact catalog/PART ledger"};
  if (scope.mass || !coefficients.prepared() || !coefficients.shells() ||
      !coefficients.domain() || !coefficients.shells()->Matches(*scope.binding,*coefficients.domain()))
    return {S::InvalidInput,"One exact physical ledger must replace shell-local combined mass"};
  if (coefficients.domain()->node_count() > limits.max_nodes ||
      scope.catalog->parent_count() > limits.max_parents)
    return {S::ResourceLimit,"Physical shell counts exceed scope"};
  if (coefficients.scope().uncovered_nodes)
    return {S::MissingSource,"Every physical node requires an explicit coefficient producer"};
  // Execution retains its complete PART/ledger proof. Its exact ledger handle
  // is copied below, so that backing is charged once through execution. The
  // failure catalog may be independently prepared with equal values; charge
  // that complete report separately without guessing shared inventory identity.
  const auto ledger_bytes = execution ? execution->forecast().owned_payload_bytes : coefficients.owned_payload_bytes();
  const auto ledger_handle = execution ? sizeof(ShellExecutionBinding) : sizeof(NodalCoefficientLedger);
  const auto failure_bytes = scope.failure->host_bytes();
  if (ledger_bytes < ledger_handle ||
      failure_bytes < sizeof(ShellBatchFailureBinding))
    return {S::ResourceLimit,"Invalid retained physical producer accounting"};
  util::BoundedArenaLayout budget(limits.max_host_bytes);
  util::ArenaRegion ignored;
  if (!budget.Append<unsigned char>(sizeof(*this)+sizeof(Impl)+64,ignored) ||
      !budget.Append<unsigned char>(ledger_bytes-ledger_handle,ignored) ||
      !budget.Append<unsigned char>(failure_bytes-sizeof(ShellBatchFailureBinding),ignored))
    return {S::ResourceLimit,"Physical shell binding and retained sources exceed byte cap"};
  try {
    auto next = std::make_shared<Impl>(execution ? *execution->coefficients() : coefficients,*scope.failure);
    if (execution) next->execution.emplace(*execution);
    next->bytes = budget.bytes();
    impl_ = std::move(next);
    return {};
  } catch (const std::bad_alloc&) {
    return {S::ResourceLimit,"Physical shell binding allocation failed"};
  }
}
const ShellExecutionBinding* ShellPhysicalBinding::execution() const noexcept {
  return impl_ && impl_->execution ? &*impl_->execution : nullptr;
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
      (impl_->execution.has_value() == other.impl_->execution.has_value() &&
       (!impl_->execution || impl_->execution->Matches(*other.impl_->execution)) &&
       impl_->coefficients.Matches(other.impl_->coefficients) &&
       impl_->failure.SameScope(other.impl_->failure)));
}
std::size_t ShellPhysicalBinding::owned_payload_bytes() const noexcept {
  return impl_ ? impl_->bytes : sizeof(*this);
}
} // namespace tl::fea
