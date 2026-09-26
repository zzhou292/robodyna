// SPDX-License-Identifier: AGPL-3.0-or-later
#include "PhysicalActivePrefix.h"
#include "../ShellPhysicalOutputRanges.h"
#include "../../solvers/NodalTrialIdentity.h"
#include "lib_utils/BoundedArena.h"
#include <algorithm>
#include <new>
namespace tl::fea {
namespace {
using S=ActivePrefixStatus;using F=ActivePrefixFamily;
ActivePrefixReport Ok(){return {S::Ok,"Every physical parent remains active"};}
ActivePrefixReport Count(std::size_t expected,std::size_t actual,std::size_t active,F family) {
  if(actual!=expected)return {S::SourceMismatch,"Activity count differs from the actual source",family};
  if(active!=actual)return {S::InactiveParent,"Physical activity transition needs contact lifecycle support",family};
  return Ok();
}
ActivePrefixReport Flags(const std::uint8_t* flags,std::size_t count,F family) {
  for(std::size_t i=0;i<count;++i)
    if(flags[i]!=1)return {flags[i]==0?S::InactiveParent:S::ReadbackFailure,
      "Physical parent is not positively active",family,i};
  return Ok();
}
}
struct PhysicalActivePrefix::Impl {
  Impl(const ShellPhysicalBinding& source,const ShellPhysicalParticipants& participants,
       ShellPhysicalPublicationIdentity identity):source(source),participants(participants),identity(identity){}
  ShellPhysicalBinding source;
  ShellPhysicalParticipants participants;
  ShellPhysicalPublicationIdentity identity;
  util::HostArena arena;
  std::uint8_t* flags=nullptr;
  ActivePrefixForecast forecast;
  ActivePrefixReport Check(FENodalState& owner,ShellBatchPublication& publication,
      const NodalTrialToken* token,const ShellPhysicalDiagnostics* candidate,
      const NodalPreparedView* view) {
    const auto bound=publication.ValidatePhysicalSources(owner,source,participants,identity);
    if(bound.status!=ShellPublicationStatus::Success)return {S::SourceMismatch,bound.message};
    ShellPhysicalDiagnostics diagnostics;
    if(candidate) {
      if(!token||!view)return {S::InvalidInput,"Prepared activity requires an actual token/view"};
      const auto checked=publication.ValidatePhysicalCandidate(owner,*token,*candidate,*view);
      if(checked.status!=ShellPublicationStatus::Success)return {S::SourceMismatch,checked.message};
      diagnostics=*candidate;
    } else {
      const auto checked=publication.CopyAcceptedPhysicalDiagnostics(owner.accepted(),&diagnostics);
      if(checked.status!=ShellPublicationStatus::Success)return {S::SourceMismatch,checked.message};
    }
    const auto family=[&](auto* batch,std::size_t count,const auto& expected,F which) -> ActivePrefixReport {
      if(!count)return Ok();
      if(!batch||count>forecast.activity_capacity)return {S::SourceMismatch,"Incomplete activity family",which};
      auto observed=expected;
      const auto report=candidate?batch->CopyPreparedParentActivity(owner,*token,expected,flags,count):
          batch->CopyAcceptedParentActivity(owner.accepted(),flags,count,&observed);
      using Status=decltype(report.status);
      if(report.status!=Status::Success)return {S::ReadbackFailure,report.message,which};
      return Flags(flags,count,which);
    };
    const auto& shells=*source.shells();
    auto report=family(participants.qeph,shells.qeph_count(),diagnostics.qeph,F::Qeph);
    if(report.status==S::Ok)report=family(participants.t3,shells.t3_count(),diagnostics.t3,F::T3);
    if(report.status==S::Ok&&diagnostics.has_qbat)
      report=Count(shells.qbat_count(),diagnostics.qbat.element_count,diagnostics.qbat.active_count,F::Qbat);
    const auto& scope=source.coefficients()->scope();
    if(report.status==S::Ok&&diagnostics.has_type25)
      report=Count(scope.type25_connections,diagnostics.type25.element_count,diagnostics.type25.active_count,F::Type25);
    if(report.status==S::Ok&&diagnostics.has_type13)
      report=Count(scope.type13_connections,diagnostics.type13.element_count,diagnostics.type13.active_count,F::Type13);
    if(report.status!=S::Ok)return report;
    // Current five solid families reject their entire cutoff/inversion trial;
    // they have no element-off transition. Beam18/TYPE45 models likewise have
    // no removal path. Their complete typed candidates were authenticated above.
    if(diagnostics.has_solids) {
      const std::size_t expected[]{scope.solid18_parents,scope.solid24_parents,scope.solid6z_parents,
        scope.solid18_law44_parents,scope.solid18_law90_parents};
      for(unsigned i=0;i<5;++i)
        if(diagnostics.solids.parent_count[i]!=expected[i])
          return {S::SourceMismatch,"Solid activity scope differs",F::Solids,i};
    }
    if(diagnostics.has_beam18&&diagnostics.beam18.parent_count!=scope.beam18_parents)
      return {S::SourceMismatch,"Beam activity scope differs",F::Beam18};
    return Ok();
  }
};
PhysicalActivePrefix::PhysicalActivePrefix()=default;
PhysicalActivePrefix::~PhysicalActivePrefix()=default;
ActivePrefixReport PhysicalActivePrefix::Preflight(const ShellPhysicalBinding& source,
    ActivePrefixLimits limits,ActivePrefixForecast& output) noexcept {
  if(!source.prepared()||!source.shells()||!source.coefficients())
    return {S::InvalidInput,"Complete physical binding is required"};
  if (!shell_physical_owner::OutputDisjoint(source,&output,sizeof(output)))
    return {S::InvalidInput,"Activity forecast output overlaps its immutable source"};
  const auto& shells=*source.shells();
  const auto count=std::max<std::size_t>({shells.qeph_count(),shells.t3_count(),1});
  constexpr std::size_t handles=sizeof(PhysicalActivePrefix)+sizeof(Impl);
  if(handles>limits.max_host_bytes||count>limits.max_host_bytes-handles)
    return {S::ResourceLimit,"Complete activity scratch exceeds its explicit host cap"};
  output={handles+count,count};return Ok();
}
ActivePrefixReport PhysicalActivePrefix::Initialize(FENodalState& owner,ShellBatchPublication& publication,
    const ShellPhysicalBinding& source,const ShellPhysicalParticipants& participants,
    const ShellPhysicalPublicationIdentity& identity,ActivePrefixLimits limits) try {
  if(impl_)return {S::AlreadyInitialized,"Active prefix binding is immutable"};
  if(owner.accepted().epoch)return {S::InvalidInput,"Active prefix must initialize before the first interval"};
  const auto bound=publication.ValidatePhysicalSources(owner,source,participants,identity);
  if(bound.status!=ShellPublicationStatus::Success)return {S::SourceMismatch,bound.message};
  ActivePrefixForecast forecast;auto report=Preflight(source,limits,forecast);
  if(report.status!=S::Ok)return report;
  auto next=std::make_unique<Impl>(source,participants,identity);
  if(!next->arena.Initialize(forecast.activity_capacity))return {S::ResourceLimit,"Activity scratch allocation failed"};
  next->flags=static_cast<std::uint8_t*>(next->arena.data());next->forecast=forecast;
  report=next->Check(owner,publication,nullptr,nullptr,nullptr);
  if(report.status!=S::Ok)return report;
  impl_=std::move(next);return Ok();
} catch(const std::bad_alloc&) {return {S::ResourceLimit,"Activity startup allocation failed"};}
ActivePrefixReport PhysicalActivePrefix::CheckAccepted(FENodalState& owner,ShellBatchPublication& publication) {
  return impl_?impl_->Check(owner,publication,nullptr,nullptr,nullptr):
      ActivePrefixReport{S::NotInitialized,"Active prefix is not initialized"};
}
ActivePrefixReport PhysicalActivePrefix::CheckPrepared(FENodalState& owner,ShellBatchPublication& publication,
    const NodalTrialToken& token,const ShellPhysicalDiagnostics& candidate,const NodalPreparedView& view) {
  return impl_?impl_->Check(owner,publication,&token,&candidate,&view):
      ActivePrefixReport{S::NotInitialized,"Active prefix is not initialized"};
}
ActivePrefixForecast PhysicalActivePrefix::allocations() const noexcept {return impl_?impl_->forecast:ActivePrefixForecast{};}
} // namespace tl::fea
