// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../ShellPhysicalPublication.h"
#include "../../assembly/ShellPhysicalBinding.h"
#include "../../constraints/NodalRigidAssemblyBinding.h"

namespace tl::fea::shell_publication_detail {
struct PhysicalState {
  PhysicalState(const ShellPhysicalBinding& p,const NodalRigidAssemblyBinding& r)
      : binding(p),rigid(r) {}
  ShellPhysicalBinding binding;
  NodalRigidAssemblyBinding rigid;
  FENodalState* owner = nullptr;
  type13::Batch* beams = nullptr;
  solids::Batch* solids = nullptr;
  ShellPhysicalPublicationIdentity identity;
  ShellPhysicalPublicationForecast forecast;
  NodalStamp accepted_stamp;
  ShellPhysicalDiagnostics accepted,candidate;
  std::size_t attachment_count = 0,witness_count = 0;
};
bool SamePhysicalDiagnostics(const ShellPhysicalDiagnostics&,
    const ShellPhysicalDiagnostics&) noexcept;
ShellPhysicalCandidates Candidates(const ShellPhysicalDiagnostics&) noexcept;
} // namespace tl::fea::shell_publication_detail
