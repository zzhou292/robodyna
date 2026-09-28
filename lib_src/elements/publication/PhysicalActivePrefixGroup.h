// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "PhysicalActivePrefix.h"
namespace tl::fea {
// Private, stack-only borrow for one callback-free native group operation.
// No public API can name, create or retain this proof. Public queries stay fresh.
struct PhysicalActivePrefix::PreparedGroupSession {
  PreparedGroupSession(FENodalState& o,ShellBatchPublication& p,const NodalTrialToken& t,
      const ShellPhysicalDiagnostics& d,const NodalPreparedView& v) noexcept
      :owner(&o),publication(&p),token(&t),candidate(&d),view(&v){}
  PreparedGroupSession(const PreparedGroupSession&)=delete;
  PreparedGroupSession& operator=(const PreparedGroupSession&)=delete;
  PreparedGroupSession(PreparedGroupSession&&)=delete;
  PreparedGroupSession& operator=(PreparedGroupSession&&)=delete;
  const FENodalState* const owner;
  const ShellBatchPublication* const publication;
  const NodalTrialToken* const token;
  const ShellPhysicalDiagnostics* const candidate;
  const NodalPreparedView* const view;
  const PhysicalActivePrefix* validated=nullptr;
};
}
