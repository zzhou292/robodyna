// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalRigidAssemblyInternal.h"
#include "NodalRigidInertiaFinalize.h"

namespace tl::fea::rigid {
NodalRigidGroupReport FinalizeAssemblyRawBody(const AssemblyRawBody& raw,AssemblyFinalBody& out) noexcept {
  if(!assembly_detail::Disjoint(&raw,sizeof raw,&out,sizeof out)||!assembly_detail::Raw(raw))
    return {NodalRigidGroupStatus::InvalidInput,"Invalid or overlapping raw rigid-body finalization"};
  NodalRigidGroupProperties value;
  value.regularization.primary_mass_kg=raw.ledger.primary_mass;
  value.regularization.primary_isotropic_inertia_kg_m2=raw.ledger.primary_inertia;
  const auto report=FinalizeInertia(raw.tensor,value,SIZE_MAX);
  if(!report)return report;
  const AssemblyFinalBody next{raw,value.principal,value.raw_principal_inertia,value.effective_tensor,value.regularization};
  out=next;return {};
}
} // namespace tl::fea::rigid
