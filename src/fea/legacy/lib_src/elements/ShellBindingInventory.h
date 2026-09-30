// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ShellBindingValues.h"

namespace tl::fea::shell_binding_detail {
// Fixed-word in-process identities. Count bounds precede this arithmetic.
inline constexpr std::size_t InventoryWords(std::size_t q,std::size_t t,
    std::size_t b,bool legacy) noexcept {
  return legacy?ShellBatchInventory::WordCount:(b?5:4)+29*q+23*t+47*b;
}
template<class Words>
void AppendProjectionMetric(Words& words,std::size_t& cursor,const qeph::ReferenceInput& input) noexcept {
  words[cursor++]=Bits(input.projection_working_length_m);
}
template<class Words>
void AppendProjectionMetric(Words&,std::size_t&,const t3::ReferenceInput&) noexcept {}
template<class Input,std::size_t N,class Words>
void AppendInventory(Words& words,
    std::size_t& cursor,std::uint64_t family,const Input& input,
    const std::array<std::size_t,N>& indices,std::uint64_t source_id,bool legacy) noexcept {
  words[cursor++]=family; words[cursor++]=N;
  if(!legacy) words[cursor++]=source_id;
  for(std::size_t i=0;i<N;++i) {
    words[cursor++]=indices[i]; words[cursor++]=input.node_ids[i];
    words[cursor++]=Bits(input.position[i].x); words[cursor++]=Bits(input.position[i].y);
    words[cursor++]=Bits(input.position[i].z);
  }
  words[cursor++]=Bits(input.density); words[cursor++]=Bits(input.thickness);
  words[cursor++]=Bits(input.young_modulus); words[cursor++]=Bits(input.poisson_ratio);
  words[cursor++]=static_cast<std::uint64_t>(input.placement);
  AppendProjectionMetric(words,cursor,input);
}
template<class Words>
void AppendQbatInventory(Words& words,std::size_t& cursor,
    const qbat::ReferenceInput& input,const std::array<std::size_t,4>& indices,
    std::uint64_t source_id) noexcept {
  // IHBE11 is an explicit formulation word, distinct from arity four/QEPH.
  AppendInventory(words,cursor,11,input.quadrilateral,indices,source_id,false);
  const auto& o=input.options;
  const int integers[]{o.ihbe,o.irep,o.ismstr,o.nptr,o.npts,o.nptt,o.layers,
      o.idrill,o.npinch,o.material_law,o.property_type,o.ithick,o.iplas};
  for(int value:integers) words[cursor++]=static_cast<std::uint64_t>(value);
  words[cursor++]=Bits(o.offset_ratio);
  words[cursor++]=Bits(o.inertia_denominator_override);
  words[cursor++]=Bits(o.membrane_viscosity);
  words[cursor++]=Bits(o.numerical_viscosity);
  words[cursor++]=Bits(input.initial_a11_pa);
}
} // namespace tl::fea::shell_binding_detail
