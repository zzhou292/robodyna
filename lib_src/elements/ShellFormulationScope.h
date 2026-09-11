// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ShellBatchFailureBinding.h"

namespace tl::fea {
class NodalMassBinding;

// Borrowed immutable HOST startup inputs, not contributor registration or an
// owner/clock/publication token. Sources must outlive this borrowed view. A
// future resident initializer must retain its own owning handles and budgets.
// At least one QBAT parent and exact complete material/failure scope are required.
struct ShellFormulationScope {
  const ShellBatchBinding* binding = nullptr;
  const ShellBatchPlasticityBinding* catalog = nullptr;
  const ShellBatchFailureBinding* failure = nullptr;
  const NodalMassBinding* mass = nullptr; // Optional complete TYPE25 composition.
};

// Read-only identity validation. Does not allocate, touch CUDA, choose limits,
// admit participants, or weaken the existing Q/T runtime rejection of QBAT.
ShellPlasticityBindingReport ValidateShellFormulationScope(const ShellFormulationScope&) noexcept;
} // namespace tl::fea
