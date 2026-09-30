// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Impl.h"
#include <new>
#include <stdexcept>
namespace tl::fea::solids::control {
Report Selection::Initialize(const SolidNodeContributions& coefficients,Input in,Limits limits) noexcept try {
  if(initialized_)return {Status::AlreadyInitialized,"Solid control selection is immutable"};
  if(!coefficients.prepared()||!coefficients.domain())return {Status::InvalidInput,"Prepared model coefficient identity required"};
  detail::Layout layout;
  auto report=detail::Plan(in,coefficients.parents().size(),limits,sizeof(Selection)+sizeof(Impl)+64,layout);
  if(!report)return report;
  if(in.profile==Profile::LegacyNoStructuralIcontrol){initialized_=true;return {};}
  if(in.source_instance_id!=coefficients.domain()->source_instance_id())return {Status::SourceMismatch,"Control source instance differs from model"};
  auto next=std::make_shared<Impl>();next->layout=layout;
  next->source_instance_id=in.source_instance_id;next->units=in.units;
  next->nvsiz=in.native_nvsiz;next->mvsiz=in.compiled_mvsiz;
  auto& storage=next->storage;
  if(!storage.arena.Initialize(layout.arena_bytes))return {Status::ResourceLimit,"Control identity arena allocation failed"};
  storage.parents=storage.arena.Construct<Parent>(layout.parents);
  storage.partitions=storage.arena.Construct<NativePartition>(layout.partitions);
  storage.packets=storage.arena.Construct<Packet>(layout.packets);
  storage.members=storage.arena.Construct<Member>(layout.members);
  if(!storage.parents||!storage.partitions||!storage.packets||!storage.members)return {Status::ResourceLimit,"Control identity arena layout is invalid"};
  detail::Scratch scratch;
  if(!scratch.arena.Initialize(layout.scratch_bytes))return {Status::ResourceLimit,"Control validation scratch allocation failed"};
  scratch.seen=scratch.arena.Construct<std::uint8_t>(layout.seen);
  if(!scratch.seen)return {Status::ResourceLimit,"Control validation layout is invalid"};
  const auto parents=coefficients.parents();
  scratch.identities.Prepare(parents.size(),[&](std::size_t i){return parents[i].source_element_id;});
  report=detail::BindParents(coefficients,in,storage,scratch,next->controlled_count);
  if(!report)return report;
  report=detail::BindPackets(in,storage,scratch);if(!report)return report;
  impl_=std::move(next);initialized_=true;return {};
} catch(const std::bad_alloc&) {return {Status::ResourceLimit,"Control selection allocation failed"};}
  catch(const std::length_error&) {return {Status::ResourceLimit,"Control selection extent overflow"};}
Profile Selection::profile()const noexcept{return impl_?Profile::SourceDeclared:Profile::LegacyNoStructuralIcontrol;}
std::uint64_t Selection::source_instance_id()const noexcept{return impl_?impl_->source_instance_id:0;}
UnitScale Selection::units()const noexcept{return impl_?impl_->units:UnitScale{};}
std::uint32_t Selection::native_nvsiz()const noexcept{return impl_?impl_->nvsiz:0;}
std::uint32_t Selection::compiled_mvsiz()const noexcept{return impl_?impl_->mvsiz:0;}
std::size_t Selection::controlled_count()const noexcept{return impl_?impl_->controlled_count:0;}
std::size_t Selection::owned_payload_bytes()const noexcept{return impl_?impl_->layout.budget.owned_bytes:sizeof(*this);}
std::size_t Selection::startup_payload_bytes()const noexcept{return impl_?impl_->layout.budget.startup_bytes:sizeof(*this);}
util::ConstView<Parent> Selection::parents()const noexcept {static const Parent empty;return {impl_?impl_->storage.parents:&empty,impl_?impl_->layout.parents.count:0};}
util::ConstView<NativePartition> Selection::partitions()const noexcept {static const NativePartition empty;return {impl_?impl_->storage.partitions:&empty,impl_?impl_->layout.partitions.count:0};}
util::ConstView<Packet> Selection::packets()const noexcept {static const Packet empty;return {impl_?impl_->storage.packets:&empty,impl_?impl_->layout.packets.count:0};}
util::ConstView<Member> Selection::members()const noexcept {static const Member empty;return {impl_?impl_->storage.members:&empty,impl_?impl_->layout.members.count:0};}
} // namespace tl::fea::solids::control
