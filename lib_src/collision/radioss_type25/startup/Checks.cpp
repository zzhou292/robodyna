// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
#include "../search/Ranges.h"
#include "../../self_contact_filters/Environment.h"
namespace tlfea::contact::radioss_type25::startup::detail {
bool Disjoint(const void* a,std::size_t an,const void* b,std::size_t bn) noexcept {
  const auto x=reinterpret_cast<std::uintptr_t>(a),y=reinterpret_cast<std::uintptr_t>(b);
  return a && b && an<=UINTPTR_MAX-x && bn<=UINTPTR_MAX-y && (x+an<=y || y+bn<=x);
}
Report CheckInput(const Input& in,const Layout&,const tl::util::HostArena& output,
    const tl::util::HostArena& scratch,const void* published,std::size_t published_bytes) noexcept {
  namespace range=search::detail;
  if (in.profile!=Profile::OrdinaryExteriorFixedMain &&
      in.profile!=Profile::OrdinaryExteriorMovingMain) return {Status::UnsupportedProfile};
  if(in.topology!=TopologyPolicy::ManifoldTwoSided && in.topology!=TopologyPolicy::NativeOrdinaryShell)
    return {Status::UnsupportedProfile};
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
  return {Status::Ok};
}
} // namespace tlfea::contact::radioss_type25::startup::detail
