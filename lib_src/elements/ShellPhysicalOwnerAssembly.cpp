// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ShellPhysicalOwner.h"
#include "../solvers/NodalNativePhysicalCoefficients.h"

namespace tl::fea::shell_physical_owner {
NodalReport BorrowAssembly(FENodalState& owner,const NodalTrialToken& token,
    const NodalStamp& expected,const NodalAssemblyView& view,std::size_t witnesses,
    NodalCinAssemblyView* output) noexcept {
  using trial_identity::Disjoint;
  if (!output || !witnesses || witnesses>NodalCinLimits{}.max_witnesses ||
      !Disjoint(output,sizeof(*output),&owner,sizeof(owner)) ||
      !Disjoint(output,sizeof(*output),&token,sizeof(token)) ||
      !Disjoint(output,sizeof(*output),&expected,sizeof(expected)) ||
      !Disjoint(output,sizeof(*output),&view,sizeof(view))) {
    return {NodalStatus::InvalidInput,"Physical contributor CIN output/extent is invalid"};
  }
  const auto authenticated=native_physical_coefficients::AuthenticateAccepted(owner,expected,view);
  if (authenticated.status!=NodalStatus::Ok) return authenticated;
  NodalCinAssemblyView next;
  const auto report=owner.BorrowCinAssembly(token,&next);
  if (report.status!=NodalStatus::Ok) return report;
  if (next.owner_id!=expected.owner_id || next.base_epoch!=expected.epoch ||
      next.attempt!=view.attempt || !next.qualification_id ||
      next.node_count!=expected.node_count || next.witness_count!=witnesses ||
      next.stream!=view.stream || !next.translational_stiffness ||
      !next.rotational_stiffness || !next.witness_activity) {
    return {NodalStatus::StaleTrial,"Physical contributor CIN destination identity differs"};
  }
  *output=next;
  return {NodalStatus::Ok,"Actual owner/token CIN stiffness destinations authenticated"};
}
} // namespace tl::fea::shell_physical_owner
