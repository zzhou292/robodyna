// SPDX-License-Identifier: AGPL-3.0-or-later
#include "shell_execution/Internal.h"
#include <new>
#include <stdexcept>

namespace tl::fea {
struct ShellExecutionBinding::Impl : shell_execution_detail::Storage {
  using Storage::Storage;
};
ShellPlasticityBindingReport ShellExecutionBinding::Preflight(
    const ShellBatchPlasticityBinding& catalog, const NodalCoefficientLedger& ledger,
    const NodalRigidAssemblyBinding& rigid, ShellExecutionLimits limits,
    ShellExecutionForecast* output) noexcept {
  using namespace shell_execution_detail;
  if (!output) return Error(Status::InvalidInput, "Missing execution forecast output");
  Layout layout;
  const auto report = Forecast(catalog, ledger, rigid, limits, sizeof(ShellExecutionBinding) + sizeof(Impl) + 64, layout);
  if (report.status == Status::Success) *output = layout.forecast;
  return report;
}
ShellPlasticityBindingReport ShellExecutionBinding::Initialize(
    const ShellBatchPlasticityBinding& catalog, const NodalCoefficientLedger& ledger,
    const NodalRigidAssemblyBinding& rigid, ShellExecutionLimits limits) noexcept try {
  using namespace shell_execution_detail;
  if (impl_) return Error(Status::AlreadyInitialized, "Shell execution binding is immutable");
  Layout layout;
  auto report = Forecast(catalog, ledger, rigid, limits, sizeof(*this) + sizeof(Impl) + 64, layout);
  if (report.status != Status::Success) return report;
  auto next = std::make_shared<Impl>(catalog, rigid);
  if (!next->arena.Initialize(layout.arena_bytes)) {
    return Error(Status::ResourceLimit, "Shell execution arena allocation failed");
  }
  next->parents = next->arena.Construct<ShellExecutionParent>(layout.parents);
  next->qeph = next->arena.Construct<std::size_t>(layout.qeph);
  next->t3 = next->arena.Construct<std::size_t>(layout.t3);
  next->qbat = next->arena.Construct<std::size_t>(layout.qbat);
  if (!next->parents || (layout.qeph.count && !next->qeph) ||
      (layout.t3.count && !next->t3) || (layout.qbat.count && !next->qbat)) {
    return Error(Status::ResourceLimit, "Shell execution arena construction failed");
  }
  report = Bind(*next);
  if (report.status != Status::Success) return report;
  next->forecast = layout.forecast;
  impl_ = std::move(next);
  return {};
} catch (const std::bad_alloc&) {
  return shell_execution_detail::Error(ShellPlasticityBindingStatus::ResourceLimit, "Shell execution allocation failed");
} catch (const std::length_error&) {
  return shell_execution_detail::Error(ShellPlasticityBindingStatus::ResourceLimit, "Shell execution index allocation overflow");
}
const ShellBatchPlasticityBinding* ShellExecutionBinding::catalog() const noexcept {
  return impl_ ? &impl_->catalog : nullptr;
}
const NodalRigidAssemblyBinding* ShellExecutionBinding::rigid() const noexcept {
  return impl_ ? &impl_->rigid : nullptr;
}
const NodalCoefficientLedger* ShellExecutionBinding::coefficients() const noexcept {
  return impl_ ? impl_->rigid.coefficients() : nullptr;
}
const ShellNodeMap* ShellExecutionBinding::mapping() const noexcept {
  return impl_ ? coefficients()->shells() : nullptr;
}
const ShellBatchBinding* ShellExecutionBinding::shells() const noexcept {
  return impl_ ? mapping()->shells() : nullptr;
}
const NodalNodeDomain* ShellExecutionBinding::domain() const noexcept {
  return impl_ ? coefficients()->domain() : nullptr;
}
tl::util::ConstView<ShellExecutionParent> ShellExecutionBinding::parents() const noexcept {
  static const ShellExecutionParent empty;
  return {impl_ ? impl_->parents : &empty, impl_ ? impl_->catalog.parent_count() : 0};
}
const ShellExecutionParent* ShellExecutionBinding::parent(std::size_t row) const noexcept {
  return impl_ && row < impl_->catalog.parent_count() ? &impl_->parents[row] : nullptr;
}
const ShellExecutionParent* ShellExecutionBinding::parent(ShellBindingFamily family, std::size_t index) const noexcept {
  if (!impl_) return nullptr;
  switch (family) {
    case ShellBindingFamily::Qeph: return index < shells()->qeph_count() ? parent(impl_->qeph[index]) : nullptr;
    case ShellBindingFamily::T3: return index < shells()->t3_count() ? parent(impl_->t3[index]) : nullptr;
    case ShellBindingFamily::Qbat: return index < shells()->qbat_count() ? parent(impl_->qbat[index]) : nullptr;
    default: return nullptr;
  }
}
ShellExecutionCounts ShellExecutionBinding::counts() const noexcept {
  return impl_ ? impl_->counts : ShellExecutionCounts{};
}
ShellExecutionForecast ShellExecutionBinding::forecast() const noexcept {
  return impl_ ? impl_->forecast : ShellExecutionForecast{};
}
bool ShellExecutionBinding::Matches(const ShellBatchPlasticityBinding& catalog,
    const NodalCoefficientLedger& ledger) const noexcept {
  return impl_ && impl_->catalog.SameScope(catalog) && coefficients()->Matches(ledger);
}
bool ShellExecutionBinding::Matches(const ShellExecutionBinding& other) const noexcept {
  return impl_ && impl_ == other.impl_;
}
} // namespace tl::fea
