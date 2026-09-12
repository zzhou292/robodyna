// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../NodalCinStorage.h"
#include "../NodalRigidGroupStorage.h"
#include "../NodalTrialIdentity.h"

namespace tl::fea::cin_physical_mains {
template<class T> inline bool Outside(const void* output, std::size_t bytes,
    const T* source, std::size_t count = 1) noexcept {
  return !count || (count <= SIZE_MAX/sizeof(T) &&
      trial_identity::Disjoint(output,bytes,source,count*sizeof(T)));
}
template<class T> inline bool Outside(const void* output, std::size_t bytes,
    const std::vector<T>& source) noexcept {
  return Outside(output,bytes,&source) && Outside(output,bytes,source.data(),source.size());
}
// Complete public retained CIN backing and compact owner metadata. Private
// headers are also excluded; caller outputs cannot overwrite a retained source
// through an independently copied handle to the same immutable model/domain.
inline bool OutsideSources(const void* output, std::size_t bytes,
    const nodal_detail::CinStorage& cin, const nodal_detail::RigidStorage* rigid,
    const std::vector<double>& staging, const std::vector<std::uint8_t>& constraints) noexcept {
  if (!Outside(output,bytes,&cin) ||
      !Outside(output,bytes,staging) || !Outside(output,bytes,constraints) ||
      !Outside(output,bytes,cin.rows) || !Outside(output,bytes,cin.witnesses) ||
      !Outside(output,bytes,cin.dependent) || !Outside(output,bytes,cin.first_witness)) return false;
  if (rigid && (!Outside(output,bytes,rigid) ||
      !Outside(output,bytes,rigid->properties) || !Outside(output,bytes,rigid->source_members) ||
      !Outside(output,bytes,rigid->groups) || !Outside(output,bytes,rigid->members) ||
      !Outside(output,bytes,rigid->member_nodes) || !Outside(output,bytes,rigid->snapshots))) return false;
  const auto& model = cin.source;
  const auto* domain = model.domain();
  const auto* classification = model.classification();
  if (!domain || !classification || !Outside(output,bytes,&model) ||
      !Outside(output,bytes,model.rows().data,model.rows().count) ||
      !Outside(output,bytes,domain) || !Outside(output,bytes,domain->nodes().data(),domain->node_count()) ||
      !Outside(output,bytes,classification) ||
      !Outside(output,bytes,classification->slaves().data,classification->slaves().count) ||
      !Outside(output,bytes,classification->interface_decode().data,classification->interface_decode().count))
    return false;
  return true;
}
inline bool OutsideSources(const void* output, std::size_t bytes,
    const nodal_detail::CinStorage& cin, const nodal_detail::RigidStorage& rigid,
    const std::vector<double>& staging, const std::vector<std::uint8_t>& constraints) noexcept {
  return OutsideSources(output, bytes, cin, &rigid, staging, constraints);
}
} // namespace tl::fea::cin_physical_mains
