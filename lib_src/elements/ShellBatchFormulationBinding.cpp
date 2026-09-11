// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ShellBatchPlasticityBindingInternal.h"

namespace tl::fea {
using namespace shell_plasticity_binding_detail;

ShellPlasticityBindingReport ShellBatchPlasticityBinding::InitializeFormulations(
    const ShellBatchBinding& binding,const ShellBatchPlasticityBindingInput& input,
    const ShellHostBindingLimits& limits) noexcept {
  if(prepared_) return Error(Status::AlreadyInitialized,"Formulation catalog is immutable");
  if(limits.max_parents>MaxShellHostParents||limits.max_nodes>MaxShellHostNodes)
    return Error(Status::ResourceLimit,"Use explicit formulation catalog limits for vehicle-sized bindings");
  return InitializeCatalogImpl(binding,input,{limits.max_parents,limits.max_nodes,
      limits.max_parents,limits.max_owned_bytes,limits.max_startup_scratch_bytes},true,true);
}

ShellPlasticityBindingReport ShellBatchPlasticityBinding::InitializeFormulationCatalog(
    const ShellBatchBinding& binding,const ShellBatchPlasticityBindingInput& input,
    const ShellPlasticityCatalogLimits& limits) noexcept {
  return InitializeCatalogImpl(binding,input,limits,true,true);
}
} // namespace tl::fea
