// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
#include "../search/Ranges.h"
#include "../../self_contact_filters/Environment.h"
#include <initializer_list>
namespace tlfea::contact::radioss_type25::startup::detail {
bool Disjoint(const void* a,std::size_t an,const void* b,std::size_t bn) noexcept {
  const auto x=reinterpret_cast<std::uintptr_t>(a),y=reinterpret_cast<std::uintptr_t>(b);
  return a && b && an<=UINTPTR_MAX-x && bn<=UINTPTR_MAX-y && (x+an<=y || y+bn<=x);
}
Report CheckInput(const Input& in,const Layout&,const tl::util::HostArena& output,
    const tl::util::HostArena& scratch,const void* published,std::size_t published_bytes) noexcept {
  namespace range=search::detail;
  if (!role_policy::Supported(in.profile,in.topology)) return {Status::UnsupportedProfile};
  if (!self_contact_filters::CompatibleHostArithmetic()) return {Status::UnsupportedArithmetic};
  if (!in.source_generation || !range::Span(in.node_source_ids,in.node_count) ||
      !range::Span(in.primary,in.primary_count) || !published ||
      reinterpret_cast<std::uintptr_t>(published)%alignof(Snapshot)) return {Status::InvalidInput};
  std::size_t position_bytes=0;
  if (!range::VectorSpan(in.positions,in.node_count,position_bytes)) return {Status::InvalidInput};
  if (in.coordinates!=Coordinates::Native && in.coordinates!=Coordinates::Si) return {Status::InvalidInput};
  if (in.coordinates==Coordinates::Si) {
    units_detail::Factors factors;
    if (!units_detail::Make(in.units,factors)) return {Status::InvalidInput};
  }
  if (!range::Span(static_cast<const std::byte*>(output.data()),output.bytes()) ||
      !range::Span(static_cast<const std::byte*>(scratch.data()),scratch.bytes()) ||
      !Disjoint(output.data(),output.bytes(),scratch.data(),scratch.bytes())) return {Status::InvalidInput};
  const void* pointers[]{&in,in.node_source_ids,in.primary,in.positions.data,published};
  const std::size_t bytes[]{sizeof(Input),in.node_count*sizeof(std::uint64_t),in.primary_count*sizeof(PrimaryFace),
      position_bytes,published_bytes};
  for (unsigned i=0;i<5;++i)
    if (!Disjoint(output.data(),output.bytes(),pointers[i],bytes[i]) ||
        !Disjoint(scratch.data(),scratch.bytes(),pointers[i],bytes[i])) return {Status::InvalidInput};
  for (unsigned i=0;i<4;++i)
    if (!Disjoint(published,published_bytes,pointers[i],bytes[i])) return {Status::InvalidInput};
  if(role_policy::Mixed(in.topology)) {
    if(in.primary_identity_count!=in.primary_count || in.shell_primary_count>in.primary_count ||
        in.raw_origin_count<in.primary_count || !range::Span(in.primary_identities,in.primary_count) ||
        !range::Span(in.raw_origins,in.raw_origin_count) ||
        !range::Span(in.raw_origin_to_primary,in.raw_origin_count))return {Status::InvalidInput};
    const auto identity_bytes=in.primary_count*sizeof(PrimaryFaceIdentity);
    if(!Disjoint(output.data(),output.bytes(),in.primary_identities,identity_bytes) ||
        !Disjoint(scratch.data(),scratch.bytes(),in.primary_identities,identity_bytes) ||
        !Disjoint(published,published_bytes,in.primary_identities,identity_bytes))return {Status::InvalidInput};
    const void* origin_pointers[]{in.raw_origins,in.raw_origin_to_primary};
    const std::size_t origin_bytes[]{in.raw_origin_count*sizeof(PrimaryFaceIdentity),in.raw_origin_count*sizeof(std::uint32_t)};
    for(unsigned i=0;i<2;++i)
      if(!Disjoint(output.data(),output.bytes(),origin_pointers[i],origin_bytes[i]) ||
          !Disjoint(scratch.data(),scratch.bytes(),origin_pointers[i],origin_bytes[i]) ||
          !Disjoint(published,published_bytes,origin_pointers[i],origin_bytes[i]))return {Status::InvalidInput};
    for(const auto* descriptor:{&output,&scratch})
      if(!Disjoint(output.data(),output.bytes(),descriptor,sizeof(*descriptor)) ||
          !Disjoint(scratch.data(),scratch.bytes(),descriptor,sizeof(*descriptor)) ||
          !Disjoint(published,published_bytes,descriptor,sizeof(*descriptor)))return {Status::InvalidInput};
  }
  return {Status::Ok};
}
} // namespace tlfea::contact::radioss_type25::startup::detail
