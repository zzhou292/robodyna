// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "SolidNodeContributions.h"
#include "NodalDomainIdentity.h"
#include <cmath>

namespace tl::fea::solid_coefficient_detail {
inline const double* Masses(const solid18::Reference& ref) noexcept {
  return ref.mass().source_nodal_mass_kg;
}
inline const double* Masses(const solid24::Reference& ref) noexcept {
  return ref.mass().source_slot_mass_kg;
}
inline const double* Masses(const solid6z::Reference& ref) noexcept {
  return ref.mass().source_slot_mass_kg;
}
inline const double* Masses(const solid18::law44::Reference& ref) noexcept {
  return ref.mass().source_nodal_mass_kg;
}
inline const double* Masses(const solid18::total_strain::Reference& ref) noexcept {
  return ref.mass().source_nodal_mass_kg;
}
// Called only after all five pointer/count pairs and total extent are checked.
// Keep the same typed visitation for identity lookup and source-slot mapping.
template<class Visitor>
auto Visit(const SolidCoefficientInput& input, std::size_t parent, Visitor visit) {
  using F = SolidCoefficientFamily;
  if (parent < input.solid18_count) return visit(input.solid18[parent], F::Solid18, 8u);
  parent -= input.solid18_count;
  if (parent < input.solid24_count) return visit(input.solid24[parent], F::Solid24, 8u);
  parent -= input.solid24_count;
  if (parent < input.solid6z_count) return visit(input.solid6z[parent], F::Solid6z, 6u);
  parent -= input.solid6z_count;
  if (parent < input.law44_count) return visit(input.law44[parent], F::Solid18Law44, 8u);
  return visit(input.law90[parent-input.law44_count], F::Solid18Law90, 8u);
}
template<class Reference>
NodalDomainReport Map(const Reference& ref, const NodalNodeDomain& domain,
    SolidCoefficientFamily family, unsigned arity, SolidCoefficientParent& out,
    std::size_t parent) noexcept {
  using S = NodalDomainStatus;
  if (!ref.prepared()) return {S::InvalidInput, "Solid reference is not prepared", parent};
  const auto& input = ref.input();
  out.family = family;
  out.source_element_id = input.source_element_id;
  out.source_part_id = input.source_part_id;
  out.source_section_id = input.source_section_id;
  out.source_material_id = input.source_material_id;
  out.node_count = arity;
  for (unsigned k = 0; k < arity; ++k) {
    const auto n = domain.Find(input.source_node_id[k]);
    if (n == SIZE_MAX) return {S::MissingSource, "Solid source node is absent from domain", parent};
    if (!nodal_domain_detail::SamePosition(input.position_m[k], domain.nodes()[n].position))
      return {S::PositionMismatch, "Solid source coordinate bits differ from domain", parent};
    const double mass = Masses(ref)[k];
    if (!std::isfinite(mass) || mass <= 0)
      return {S::InvalidInput, "Solid prepared source-slot mass is not positive finite", parent};
    out.source_node_id[k] = input.source_node_id[k];
    out.domain_node[k] = n;
    out.mass_kg[k] = mass;
  }
  return {};
}
inline bool Same(const SolidCoefficientParent& a, const SolidCoefficientParent& b) noexcept {
  if (a.family != b.family || a.node_count != b.node_count ||
      a.source_element_id != b.source_element_id || a.source_part_id != b.source_part_id ||
      a.source_section_id != b.source_section_id || a.source_material_id != b.source_material_id)
    return false;
  for (unsigned k = 0; k < a.node_count; ++k)
    if (a.source_node_id[k] != b.source_node_id[k] || a.domain_node[k] != b.domain_node[k] ||
        std::memcmp(&a.mass_kg[k], &b.mass_kg[k], sizeof(double)) != 0) return false;
  return true;
}
}  // namespace tl::fea::solid_coefficient_detail
