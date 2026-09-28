// SPDX-License-Identifier: AGPL-3.0-or-later
#include "GroupSealSession.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include <array>
namespace tlfea::contact::radioss_type25 {
namespace fe=tl::fea;
TransactionGroupReport Transaction::SealCandidateGroup(Transaction* const* members,
    std::size_t count,fe::ShellBatchPublication& publication,fe::FENodalState& owner,
    const fe::NodalTrialToken& token,const fe::NodalPreparedView& view,
    const fe::ShellPhysicalDiagnostics& physical,
    fe::ShellPhysicalScratchParticipationReceipt* output,std::size_t output_count) noexcept {
  constexpr auto maximum=fe::MaxNativeContactInterfaces;
  std::array<Transaction*,maximum> borrowed{};
  const auto fail=[&](TransactionReport report,std::size_t index) {
    owner.Discard();publication.DiscardNativeCandidateGroup(owner);
    for(auto* member:borrowed)if(member && member->impl_ && member->impl_->owner==&owner &&
        member->impl_->publication==&publication)member->impl_->DiscardLocal();
    return TransactionGroupReport{report,index};
  };
  if(!members || !count || count>maximum || output_count!=count || !output ||
      reinterpret_cast<std::uintptr_t>(members)%alignof(Transaction*) ||
      reinterpret_cast<std::uintptr_t>(output)%alignof(fe::ShellPhysicalScratchParticipationReceipt))
    return fail({TransactionStatus::InvalidInput,"Invalid bounded native candidate group"},SIZE_MAX);
  const auto bytes=count*sizeof(*output);
  using fe::trial_identity::Disjoint;
  if(!Disjoint(output,bytes,members,count*sizeof(*members)) ||
      !Disjoint(output,bytes,&token,sizeof(token)) || !Disjoint(output,bytes,&view,sizeof(view)) ||
      !Disjoint(output,bytes,&physical,sizeof(physical)))
    return fail({TransactionStatus::InvalidInput,"Native group receipt output overlaps its input"},SIZE_MAX);
  for(std::size_t i=0;i<count;++i)borrowed[i]=members[i];
  std::array<fe::ShellPhysicalScratchParticipationReceipt,maximum> staged;
  static_assert(sizeof(GroupSealSession)+sizeof(staged)+sizeof(borrowed)<=4096,
      "Native group borrow and receipt staging must stay within the fixed stack bound");
  GroupSealSession session(owner,publication,token,physical,view,count);
  // No caller callback or physical producer occurs inside this bounded loop.
  // Successful selector staging and issuer sealing are host-only and cannot
  // modify the observed material backing. The private proof dies on return.
  for(std::size_t i=0;i<count;++i) {
    auto* member=borrowed[i];
    if(!member)return fail({TransactionStatus::NotInitialized,"Native transaction is not initialized"},i);
    if(member->impl_ && member->impl_->issuer.configured() && (!member->impl_->OutputDisjoint(output,bytes) ||
        !Disjoint(output,bytes,member,sizeof(*member))))
      return fail({TransactionStatus::InvalidInput,"Native group receipt output overlaps retained state"},i);
    const auto report=member->SealCandidateImpl(owner,token,view,physical,&staged[i],&session,i);
    if(report.status!=TransactionStatus::Ok)return fail(report,i);
  }
  for(std::size_t i=0;i<count;++i)output[i]=staged[i];
  return {{TransactionStatus::Ok,"OK"},SIZE_MAX};
}
}
