// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Storage.h"
#include "lib_src/elements/publication/PhysicalActivePrefixGroup.h"
namespace tlfea::contact::radioss_type25 {
struct Transaction::GroupSealSession {
  GroupSealSession(tl::fea::FENodalState& owner,tl::fea::ShellBatchPublication& p,
      const tl::fea::NodalTrialToken& token,const tl::fea::ShellPhysicalDiagnostics& candidate,
      const tl::fea::NodalPreparedView& view,std::size_t n) noexcept
      :activity(owner,p,token,candidate,view),publication(p),count(n){}
  tl::fea::PhysicalActivePrefix::PreparedGroupSession activity;
  tl::fea::ShellBatchPublication& publication;
  const std::size_t count;
};
}
