// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ShellBatchPublication.h"

namespace tl::fea::shell_publication_detail {
using S=ShellPublicationStatus;
struct Model;
void PopulateKineticModel(const ShellBatchBinding&,const NodalMassBinding*,Model&) noexcept;
ShellPublicationReport Ok() noexcept;
ShellPublicationReport Nodal(const NodalReport&) noexcept;
ShellPublicationReport Connector(const type25::BatchReport&) noexcept;
ShellPublicationReport Qbat(const qbat::BatchReport&) noexcept;
bool SameDiagnostics(const ShellBatchDiagnostics&,const ShellBatchDiagnostics&) noexcept;
template<class D> bool UnavailableKinetic(const D& d) noexcept {
  return !d.kinetic_available&&d.kinetic_translation==0&&d.kinetic_rotation==0&&
      d.kinetic_physical_isotropic==0&&d.kinetic_added_isotropic==0;
}
ShellPublicationReport InitialMovingKinetic(FENodalState&,const ShellBatchBinding&,
    const NodalMassBinding*,const ShellBatchStartup&,const NodalStamp&,ShellBatchKinetic&);
} // namespace tl::fea::shell_publication_detail
