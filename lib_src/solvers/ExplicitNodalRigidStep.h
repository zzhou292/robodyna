// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ExplicitNodalStep.h"
namespace tl::fea {
// One sealed assembly, one owner transaction. Requires attached plain groups
// and the case-qualified staggered admission/validation path. Free nodes keep
// ordinary TL isotropic motion; members use the native rigid recurrence. This
// operation owns no state, timestep or acceptance/publication mechanism.
NodalReport AdvanceStaggeredRigidGroups(FENodalState&,const NodalTrialToken&,
                                      const NodalStaggeredHistoryAdmission&);
} // namespace tl::fea
