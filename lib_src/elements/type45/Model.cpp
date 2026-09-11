// SPDX-License-Identifier: AGPL-3.0-or-later
#include "model/Internal.h"
#include <new>
#include <stdexcept>

namespace tl::fea::type45 {
struct Model::Impl {
  explicit Impl(const NodalRigidAssemblyBinding& value):rigid(value) {}
  NodalRigidAssemblyBinding rigid;
  util::HostArena arena;
  Joint* joints=nullptr;
  model_detail::Layout layout;
};
ModelReport Model::Initialize(const NodalRigidAssemblyBinding& rigid,ModelInput input,ModelLimits limits) noexcept try {
  using namespace model_detail;
  if(impl_) return Error(ModelStatus::AlreadyInitialized,"Joint model is immutable");
  Layout layout;
  auto checked=Preflight(rigid,input,limits,sizeof(Model)+sizeof(Impl)+64,layout);
  if(!checked) return checked;
  Index identities;
  identities.Prepare(input.joints.size(),[&](std::size_t i){return input.joints[i].geometry.source_joint_id;});
  auto next=std::make_shared<Impl>(rigid);
  if(!next->arena.Initialize(layout.arena) || !(next->joints=next->arena.Construct<Joint>(layout.joints)))
    return Error(ModelStatus::ResourceLimit,"Joint model arena allocation failed");
  next->layout=layout;
  for(std::size_t i=0;i<input.joints.size();++i) {
    if(identities.First(input.joints[i].geometry.source_joint_id)!=i)
      return Error(ModelStatus::DuplicateIdentity,"Repeated original joint source identity",i);
    checked=PrepareJoint(rigid,input.joints[i],next->joints[i]);
    if(!checked) {checked.joint=i; return checked;}
  }
  impl_=std::move(next);
  return {};
} catch(const std::bad_alloc&) {
  return {ModelStatus::ResourceLimit,"Joint model host allocation failed"};
} catch(const std::length_error&) {
  return {ModelStatus::ResourceLimit,"Joint model startup index extent overflow"};
}
std::uint64_t Model::source_instance_id() const noexcept {return impl_?impl_->rigid.domain()->source_instance_id():0;}
const NodalRigidAssemblyBinding* Model::rigid_binding() const noexcept {return impl_?&impl_->rigid:nullptr;}
const NodalNodeDomain* Model::domain() const noexcept {return impl_?impl_->rigid.domain():nullptr;}
util::ConstView<Joint> Model::joints() const noexcept {
  static const Joint empty;
  return {impl_?impl_->joints:&empty,impl_?impl_->layout.joints.count:0};
}
bool Model::SharesStorage(const Model& other) const noexcept {return impl_ && impl_==other.impl_;}
std::size_t Model::owned_payload_bytes() const noexcept {return impl_?impl_->layout.owned:sizeof(Model);}
std::size_t Model::startup_payload_bytes() const noexcept {return impl_?impl_->layout.startup:sizeof(Model);}
} // namespace tl::fea::type45
