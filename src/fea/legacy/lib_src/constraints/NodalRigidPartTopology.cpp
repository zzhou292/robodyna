// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalRigidPartTopologyInternal.h"
#include <new>
#include <stdexcept>

namespace tl::fea::rigid {
NodalRigidPartTopology::NodalRigidPartTopology()=default;
NodalRigidPartTopology::~NodalRigidPartTopology()=default;
PartTopologyReport NodalRigidPartTopology::Initialize(const PartTopologyInput& in) noexcept {
  using namespace topology_detail;
  if(impl_)return Fail(Status::AlreadyInitialized,"Rigid PART topology is immutable after initialization");
  Counts counts;const auto preflight=Preflight(in,sizeof(Impl),counts);if(!preflight)return preflight;
  try {
    auto next=std::make_unique<Impl>();
    next->parts=std::make_unique<PartTopologyPart[]>(in.part_count);
    if(in.extra_count)next->extras=std::make_unique<PartTopologyExtra[]>(in.extra_count);
    if(in.merge_count)next->merges=std::make_unique<PartTopologyMerge[]>(in.merge_count);
    next->roots=std::make_unique<PartTopologyRoot[]>(in.part_count-in.merge_count);
    next->original=std::make_unique<SourceNodeId[]>(counts.members);
    next->root_members=std::make_unique<SourceNodeId[]>(counts.members);
    next->expected=std::make_unique<SourceNodeId[]>(counts.members);
    if(in.other_rigid_member_count)next->other=std::make_unique<SourceNodeId[]>(in.other_rigid_member_count);
    Index parts,sets,members,expected,other;
    auto report=CopyParts(in,*next,parts,sets);if(!report)return report;
    members.Prepare(counts.members,[&](std::size_t i){return next->original[i];});
    report=CheckMembers(in,next->original.get(),members,expected,other);if(!report)return report;
    report=BuildRoots(in,*next,parts);if(!report)return report;
    std::copy_n(in.expected_members,counts.members,next->expected.get());
    if(in.other_rigid_member_count)std::copy_n(in.other_rigid_members,in.other_rigid_member_count,next->other.get());
    next->source_instance=in.source_instance_id;
    next->nparts=in.part_count;next->nextras=in.extra_count;next->nmerges=in.merge_count;
    next->nroots=in.part_count-in.merge_count;next->nmembers=counts.members;next->nother=in.other_rigid_member_count;
    next->owned_bytes=counts.owned;next->startup_bytes=counts.startup;
    impl_=std::move(next);return {};
  } catch(const std::bad_alloc&) {return Fail(Status::ResourceLimit,"Rigid PART topology startup allocation failed");}
    catch(const std::length_error&) {return Fail(Status::ResourceLimit,"Rigid PART topology allocation size is not representable");}
}
bool NodalRigidPartTopology::prepared() const noexcept {return bool(impl_);}
std::uint64_t NodalRigidPartTopology::source_instance_id() const noexcept {return impl_?impl_->source_instance:0;}
std::size_t NodalRigidPartTopology::part_count() const noexcept {return impl_?impl_->nparts:0;}
std::size_t NodalRigidPartTopology::extra_count() const noexcept {return impl_?impl_->nextras:0;}
std::size_t NodalRigidPartTopology::merge_count() const noexcept {return impl_?impl_->nmerges:0;}
std::size_t NodalRigidPartTopology::root_count() const noexcept {return impl_?impl_->nroots:0;}
std::size_t NodalRigidPartTopology::member_count() const noexcept {return impl_?impl_->nmembers:0;}
std::size_t NodalRigidPartTopology::other_rigid_member_count() const noexcept {return impl_?impl_->nother:0;}
std::size_t NodalRigidPartTopology::owned_payload_bytes() const noexcept {return impl_?impl_->owned_bytes:0;}
std::size_t NodalRigidPartTopology::startup_payload_bytes() const noexcept {return impl_?impl_->startup_bytes:0;}
const PartTopologyPart* NodalRigidPartTopology::parts() const noexcept {return impl_?impl_->parts.get():nullptr;}
const PartTopologyExtra* NodalRigidPartTopology::extras() const noexcept {return impl_?impl_->extras.get():nullptr;}
const PartTopologyMerge* NodalRigidPartTopology::merges() const noexcept {return impl_?impl_->merges.get():nullptr;}
const PartTopologyRoot* NodalRigidPartTopology::roots() const noexcept {return impl_?impl_->roots.get():nullptr;}
const SourceNodeId* NodalRigidPartTopology::original_members() const noexcept {return impl_?impl_->original.get():nullptr;}
const SourceNodeId* NodalRigidPartTopology::root_members() const noexcept {return impl_?impl_->root_members.get():nullptr;}
const SourceNodeId* NodalRigidPartTopology::expected_members() const noexcept {return impl_?impl_->expected.get():nullptr;}
const SourceNodeId* NodalRigidPartTopology::other_rigid_members() const noexcept {return impl_?impl_->other.get():nullptr;}
} // namespace tl::fea::rigid
