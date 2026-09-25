// SPDX-License-Identifier: AGPL-3.0-or-later
#include "FENodalStateStorage.h"
#include "NodalRigidGroupStorage.h"
#include "../assembly/NodalDomainIdentity.h"
#include <cstring>

namespace tl::fea {
namespace {
bool Bits(double a, double b) noexcept { return std::memcmp(&a,&b,sizeof(double)) == 0; }
bool Same(const RigidBindingGroup& a, const RigidBindingGroup& b) noexcept {
  using nodal_domain_detail::SamePosition;
  if (a.source_kind != b.source_kind || a.source_id != b.source_id ||
      a.dependent_coefficients != b.dependent_coefficients ||
      a.source_node_set_id != b.source_node_set_id || a.member_offset != b.member_offset ||
      a.member_count != b.member_count || !Bits(a.mass_kg,b.mass_kg) ||
      !SamePosition(a.center,b.center) || !SamePosition(a.principal.inertia,b.principal.inertia)) return false;
  for (unsigned i=0;i<9;++i) if (!Bits(a.principal.axes.v[i],b.principal.axes.v[i])) return false;
  return true;
}
bool Same(const RigidBindingMember& a, const RigidBindingMember& b) noexcept {
  return a.source_node_id == b.source_node_id && a.domain_node == b.domain_node &&
      nodal_domain_detail::SamePosition(a.position,b.position) && Bits(a.mass_kg,b.mass_kg) &&
      Bits(a.isotropic_inertia_kg_m2,b.isotropic_inertia_kg_m2);
}
}
NodalReport FENodalState::ValidateRigidAssemblyBinding(const NodalRigidAssemblyBinding& binding) const noexcept {
  if (!impl_) return {NodalStatus::NotInitialized,"Owner is not initialized"};
  const auto& state=*impl_;
  if (!state.usable) return {NodalStatus::DeviceFailure,"CUDA owner is poisoned"};
  if (binding.explicitly_empty()) {
    if (!state.empty_rigid_source||state.rigid_groups||
        !SameRigidGroupInfo(state.stamp.rigid_groups,NodalRigidGroupInfo{})||
        binding.domain()->node_count()!=state.stamp.node_count||
        !binding.domain()->SharesStorage(*state.empty_rigid_source->domain())||
        !binding.coefficients()->Matches(*state.empty_rigid_source->coefficients()))
      return {NodalStatus::InvalidInput,"Empty rigid binding is not the owner's retained physical source"};
    return {NodalStatus::Ok,"Explicit empty rigid source matches the actual physical owner"};
  }
  if (!binding.prepared() || !state.rigid_groups || !state.stamp.rigid_groups.part_group_count ||
      binding.domain()->node_count()!=state.stamp.node_count) {
    return {NodalStatus::InvalidInput,"Owner requires its complete prepared rigid assembly binding"};
  }
  const auto& actual=*state.rigid_groups;
  const NodalRigidGroupInfo expected{binding.parts()->topology()->source_instance_id(),
      binding.groups().size(),binding.members().size(),binding.parts()->roots().size(),
      binding.plain_source_instance_id()};
  const auto units=binding.parts()->source_units();
  if (!SameRigidGroupInfo(actual.info,expected) ||
      !Bits(actual.units.mass_to_kg,units.mass_to_kg) ||
      !Bits(actual.units.length_to_m,units.length_to_m) ||
      actual.properties.size()!=binding.groups().size() ||
      actual.source_members.size()!=binding.members().size()) {
    return {NodalStatus::InvalidInput,"Rigid source identities, units or complete counts differ"};
  }
  for (std::size_t i=0;i<actual.properties.size();++i)
    if (!Same(actual.properties[i],binding.groups()[i]))
      return {NodalStatus::InvalidInput,"Rigid source primary or native member range differs"};
  for (std::size_t i=0;i<actual.source_members.size();++i)
    if (!Same(actual.source_members[i],binding.members()[i]))
      return {NodalStatus::InvalidInput,"Rigid source member identity, order or coefficients differ",
          static_cast<std::uint32_t>(actual.source_members[i].domain_node)};
  return {NodalStatus::Ok,"Complete retained rigid assembly metadata matches"};
}
} // namespace tl::fea
