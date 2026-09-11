// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalRigidPartAssemblyInternal.h"
#include <new>
#include <stdexcept>

namespace tl::fea::rigid {
struct NodalRigidPartAssemblyModel::Impl:part_assembly_detail::Storage {
  using Storage::Storage;
};
PartAssemblyReport NodalRigidPartAssemblyModel::Initialize(const NodalRigidPartTopology& topology,
    const NodalCoefficientLedger& coefficients,NodalRigidSourceUnits units,PartAssemblyLimits limits) noexcept try {
  using namespace part_assembly_detail;
  if(impl_) return Fail(S::AlreadyInitialized,"Rigid PART aggregate is immutable");
  Layout layout;
  auto report=Preflight(topology,coefficients,units,limits,sizeof(*this)+sizeof(Impl)+64,layout);
  if(!report) return report;
  auto next=std::make_shared<Impl>(coefficients);
  report=Clone(topology,next->topology);
  if(!report) return report;
  if(!next->arena.Initialize(layout.arena)||
      !(next->members=next->arena.Construct<PartAssemblyMember>(layout.members))||
      !(next->originals=next->arena.Construct<PartAssemblyOriginal>(layout.originals))||
      !(next->roots=next->arena.Construct<PartAssemblyRoot>(layout.roots))||
      !(next->lookup=next->arena.Construct<std::size_t>(layout.lookup)))
    return Fail(S::ResourceLimit,"Rigid PART aggregate arena allocation failed");
  next->units=units;
  report=MapAndAssemble(*next);
  if(!report) return report;
  next->retained=layout.retained;
  next->startup=layout.startup;
  impl_=std::move(next);
  return {};
} catch(const std::bad_alloc&) {
  return part_assembly_detail::Fail(PartAssemblyStatus::ResourceLimit,"Rigid PART aggregate allocation failed");
} catch(const std::length_error&) {
  return part_assembly_detail::Fail(PartAssemblyStatus::ResourceLimit,"Rigid PART aggregate allocation overflow");
}
const NodalRigidPartTopology* NodalRigidPartAssemblyModel::topology() const noexcept {
  return impl_?&impl_->topology:nullptr;
}
const NodalCoefficientLedger* NodalRigidPartAssemblyModel::coefficients() const noexcept {
  return impl_?&impl_->coefficients:nullptr;
}
NodalRigidSourceUnits NodalRigidPartAssemblyModel::source_units() const noexcept {
  return impl_?impl_->units:NodalRigidSourceUnits{};
}
tl::util::ConstView<PartAssemblyMember> NodalRigidPartAssemblyModel::members() const noexcept {
  static const PartAssemblyMember empty;
  return {impl_?impl_->members:&empty,impl_?impl_->topology.member_count():0};
}
tl::util::ConstView<PartAssemblyOriginal> NodalRigidPartAssemblyModel::original_bodies() const noexcept {
  static const PartAssemblyOriginal empty;
  return {impl_?impl_->originals:&empty,impl_?impl_->topology.part_count():0};
}
tl::util::ConstView<PartAssemblyRoot> NodalRigidPartAssemblyModel::roots() const noexcept {
  static const PartAssemblyRoot empty;
  return {impl_?impl_->roots:&empty,impl_?impl_->topology.root_count():0};
}
std::size_t NodalRigidPartAssemblyModel::RootForDomainNode(std::size_t node) const noexcept {
  if(!impl_||node>=impl_->coefficients.domain()->node_count()) return SIZE_MAX;
  const auto* begin=impl_->lookup;
  const auto* end=begin+impl_->topology.member_count();
  const auto* found=std::lower_bound(begin,end,node,[&](std::size_t row,std::size_t value) {
    return impl_->members[row].domain_node<value;
  });
  if(found==end||impl_->members[*found].domain_node!=node) return SIZE_MAX;
  return impl_->topology.parts()[impl_->members[*found].part_index].root_index;
}
bool NodalRigidPartAssemblyModel::Matches(const NodalRigidPartTopology& t,
    const NodalCoefficientLedger& c,NodalRigidSourceUnits u) const noexcept {
  using namespace part_assembly_detail;
  return impl_&&Same(impl_->units.mass_to_kg,u.mass_to_kg)&&
    Same(impl_->units.length_to_m,u.length_to_m)&&impl_->coefficients.Matches(c)&&
    SameTopology(impl_->topology,t);
}
bool NodalRigidPartAssemblyModel::Matches(const NodalRigidPartAssemblyModel& other) const noexcept {
  return impl_&&other.impl_&&(impl_==other.impl_||
    Matches(other.impl_->topology,other.impl_->coefficients,other.impl_->units));
}
std::size_t NodalRigidPartAssemblyModel::owned_payload_bytes() const noexcept {
  return impl_?impl_->retained:sizeof(*this);
}
std::size_t NodalRigidPartAssemblyModel::startup_payload_bytes() const noexcept {
  return impl_?impl_->startup:sizeof(*this);
}
} // namespace tl::fea::rigid
