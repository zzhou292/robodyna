// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalRigidAssemblyBindingInternal.h"

namespace tl::fea {
struct NodalRigidAssemblyBinding::Impl:rigid_binding_detail::Storage {
  explicit Impl(const rigid::NodalRigidPartAssemblyModel& p):Storage(p) {}
};
RigidBindingReport NodalRigidAssemblyBinding::Initialize(const rigid::NodalRigidPartAssemblyModel& parts,
    const NodalRigidGroupModel* plain,RigidBindingLimits limits) noexcept {
  using namespace rigid_binding_detail;
  if(impl_)return Fail(S::AlreadyInitialized,"Rigid binding is immutable");
  Layout layout;
  auto report=Forecast(parts,plain,limits,sizeof(NodalRigidAssemblyBinding)+sizeof(Impl)+64,layout);
  if(!report)return report;
  try {
    auto next=std::make_shared<Impl>(parts);
    if(!next->arena.Initialize(layout.arena)||
        !(next->groups=next->arena.Construct<RigidBindingGroup>(layout.groups))||
        !(next->members=next->arena.Construct<RigidBindingMember>(layout.members))||
        !(next->lookup=next->arena.Construct<std::size_t>(layout.lookup)))
      return Fail(S::ResourceLimit,"Combined rigid binding arena allocation failed");
    next->group_count=layout.groups.count;next->member_count=layout.members.count;
    next->owned=layout.owned;next->startup=layout.startup;
    next->plain_source=plain?plain->source_instance_id():0;
    report=Bind(*next,plain);
    if(!report)return report;
    impl_=std::move(next);return {};
  }catch(const std::bad_alloc&) {
    return Fail(S::ResourceLimit,"Combined rigid binding startup allocation failed");
  }
}
const rigid::NodalRigidPartAssemblyModel* NodalRigidAssemblyBinding::parts() const noexcept {
  return impl_?&impl_->parts:nullptr;
}
const NodalCoefficientLedger* NodalRigidAssemblyBinding::coefficients() const noexcept {
  return impl_?impl_->parts.coefficients():nullptr;
}
const NodalNodeDomain* NodalRigidAssemblyBinding::domain() const noexcept {
  return impl_?impl_->parts.coefficients()->domain():nullptr;
}
std::uint64_t NodalRigidAssemblyBinding::plain_source_instance_id() const noexcept {
  return impl_?impl_->plain_source:0;
}
tl::util::ConstView<RigidBindingGroup> NodalRigidAssemblyBinding::groups() const noexcept {
  static const RigidBindingGroup empty;
  return {impl_?impl_->groups:&empty,impl_?impl_->group_count:0};
}
tl::util::ConstView<RigidBindingMember> NodalRigidAssemblyBinding::members() const noexcept {
  static const RigidBindingMember empty;
  return {impl_?impl_->members:&empty,impl_?impl_->member_count:0};
}
const RigidBindingMember* NodalRigidAssemblyBinding::FindMember(std::size_t node) const noexcept {
  if(!impl_)return nullptr;
  std::size_t low=0,high=impl_->member_count;
  while(low<high) {
    const auto mid=low+(high-low)/2;
    if(impl_->members[impl_->lookup[mid]].domain_node<node)low=mid+1;
    else high=mid;
  }
  if(low==impl_->member_count)return nullptr;
  const auto& found=impl_->members[impl_->lookup[low]];
  return found.domain_node==node?&found:nullptr;
}
std::size_t NodalRigidAssemblyBinding::owned_payload_bytes() const noexcept {
  return impl_?impl_->owned:sizeof(NodalRigidAssemblyBinding);
}
std::size_t NodalRigidAssemblyBinding::startup_payload_bytes() const noexcept {
  return impl_?impl_->startup:sizeof(NodalRigidAssemblyBinding);
}
} // namespace tl::fea
