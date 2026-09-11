// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ShellBatchStartup.h"
#include "../assembly/NodalCoefficientLedger.h"
#include "../solvers/NodalCinRuntime.h"
#include "../../lib_utils/BoundedArena.h"

namespace tl::fea::shell_physical_owner {
// Temporary initial readback only, never a second owner or accepted history.
struct ProofLayout {
  util::ArenaRegion kinematics,coefficients;
  std::size_t bytes=0;
};
bool ForecastProof(std::size_t nodes,std::size_t attachments,std::size_t cap,
    ProofLayout&) noexcept;
// Requires fresh collocated staggered state and the exact complete CIN model.
// The full initial ledger is checked against raw M/J, not constrained inverses.
NodalReport AuthenticateInitial(const NodalCoefficientLedger&,FENodalState&,
    const NodalStamp&,const ShellBatchStartup&,const NodalCinWitnessSource&,
    const ProofLayout&);
// Later CIN coefficients are owner state and may differ from the initial ledger.
// This authenticates the actual token/view and stages the stiffness destinations.
NodalReport BorrowAssembly(FENodalState&,const NodalTrialToken&,const NodalStamp&,
    const NodalAssemblyView&,std::size_t witness_count,NodalCinAssemblyView*) noexcept;
} // namespace tl::fea::shell_physical_owner
