#pragma once
#include "ShellBatchFailure.h"
#include "ShellBatchPlasticityBinding.h"
#include <memory>

namespace tl::fea {
struct ShellFailureParentInput {
  ShellPlasticityParentInput source;
  ShellFailurePolicy policy = ShellFailurePolicy::None;
  sections::ConstantFailureParameters constant{};
  sections::ShellLayeredTab1Parameters tab1{};
};
// Immutable declaration only. Every catalog parent is supplied in the exact
// original catalog input order, including non-failing LAW1/LAW44 parents.
// The retained catalog owns all borrowed curves; no extra owner/clock exists.
class ShellBatchFailureBinding {
 public:
  ShellPlasticityBindingReport Initialize(const ShellBatchPlasticityBinding&,
      const ShellFailureParentInput*, std::size_t, const ShellBatchFailureLimits& = {}) noexcept;
  // Complete explicit execution catalog, including canonical None rigid rows.
  // This entry also permits an all-None execution scope; it adds no state.
  ShellPlasticityBindingReport InitializeExecution(const ShellBatchPlasticityBinding&,
      const ShellFailureParentInput*, std::size_t, const ShellBatchFailureLimits& = {}) noexcept;
  bool prepared() const noexcept { return bool(data_); }
  bool Matches(const ShellBatchPlasticityBinding&) const noexcept;
  bool SameScope(const ShellBatchFailureBinding&) const noexcept;
  const ShellBatchPlasticityBinding* catalog() const noexcept;
  const ShellFailureParentInput* parent(std::size_t input_index) const noexcept;
  const ShellFailureParentInput* parent(ShellBindingFamily, std::size_t family_index) const noexcept;
  // Owned payload plus bounded allocation-control allowances; excludes allocator overhead/RSS.
  std::size_t host_bytes() const noexcept;
  std::size_t parent_count() const noexcept;
 private:
  ShellPlasticityBindingReport InitializeImpl(const ShellBatchPlasticityBinding&,
      const ShellFailureParentInput*, std::size_t, const ShellBatchFailureLimits&, bool execution) noexcept;
  struct Data;
  std::shared_ptr<const Data> data_;
};
} // namespace tl::fea
