// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ShellFormulationScope.h"
#include "ShellBatchPlasticityBindingInternal.h"
#include "../assembly/NodalMassBinding.h"

namespace tl::fea {
ShellPlasticityBindingReport ValidateShellFormulationScope(const ShellFormulationScope& scope) noexcept {
  using namespace shell_plasticity_binding_detail;
  if (!scope.binding || !scope.catalog || !scope.failure ||
      !scope.binding->prepared() || !scope.binding->qbat_count() ||
      !scope.catalog->formulation_sections() || scope.catalog->execution_sections() ||
      !scope.failure->prepared()) {
    return Error(Status::InvalidInput,"Complete prepared QBAT formulation inputs are required");
  }
  if (!scope.catalog->Matches(*scope.binding) || !scope.failure->Matches(*scope.catalog)) {
    return Error(Status::IdentityMismatch,"Formulation geometry, material and failure scopes differ");
  }
  if (scope.mass && !scope.mass->Matches(*scope.binding)) {
    return Error(Status::IdentityMismatch,"Combined mass differs from complete formulation inventory");
  }
  return {};
}
ShellPlasticityBindingReport ValidateShellExecutionScope(const ShellFormulationScope& scope) noexcept {
  using namespace shell_plasticity_binding_detail;
  if (!scope.binding || !scope.catalog || !scope.failure || scope.mass ||
      !scope.binding->prepared() || !scope.catalog->execution_sections() || !scope.failure->prepared()) {
    return Error(Status::InvalidInput, "Complete explicit execution catalog and failure declarations are required");
  }
  if (!scope.catalog->Matches(*scope.binding) || !scope.failure->Matches(*scope.catalog)) {
    return Error(Status::IdentityMismatch, "Execution geometry, material and failure scopes differ");
  }
  return {};
}
} // namespace tl::fea
