// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
namespace tlfea::contact::radioss_type25::surface_interface::detail {
namespace range=search::detail;
using source_nodal::detail::Range;
using source_nodal::detail::Disjoint;
namespace {
bool Output(tl::util::BoundedArenaLayout& arena,std::size_t count,std::size_t solids,OutputLayout& out) {
  return arena.Append<startup::PrimaryFace>(count,out.primary) &&
      arena.Append<startup::PrimaryFaceIdentity>(count,out.identities) &&
      arena.Append<RawClassification>(count,out.classifications) &&
      arena.Append<std::uint32_t>(count,out.raw_to_primary) &&
      arena.Append<std::uint32_t>(count,out.primary_to_raw) &&
      arena.Append<startup::PrimaryFaceIdentity>(count,out.raw_origins) &&
      arena.Append<std::uint8_t>(solids,out.solid_flags);
}
template<class T> T* Values(tl::util::HostArena& arena,tl::util::ArenaRegion region) {
  return region.count?arena.Construct<T>(region):nullptr;
}
}
Report FromPhysical(source_surfaces::Report input) noexcept {
  Report out;
  switch(input.status) {
    case source_surfaces::Status::Ok:out.status=Status::Ok;break;
    case source_surfaces::Status::ResourceLimit:out.status=Status::ResourceLimit;break;
    case source_surfaces::Status::UnsupportedArithmetic:out.status=Status::UnsupportedArithmetic;break;
    case source_surfaces::Status::UnsupportedProfile:out.status=Status::UnsupportedProfile;break;
    default:out.status=Status::InvalidInput;break;
  }
  out.context_row=input.row;out.node=input.node;return out;
}
Report MakeLayout(const Input& in,Limits cap,Layout& out) noexcept {
  const Limits hard;
  const auto& p=in.physical;
  if(!cap.nodes||cap.nodes>hard.nodes||cap.solids>hard.solids||cap.shells>hard.shells||
      !cap.faces||cap.faces>hard.faces||!cap.output_bytes||cap.output_bytes>hard.output_bytes||
      !cap.scratch_bytes||cap.scratch_bytes>hard.scratch_bytes||p.node_count>cap.nodes||
      p.solid_count>cap.solids||p.quad_count>cap.shells||p.triangle_count>cap.shells-p.quad_count||
      in.raw_face_count>cap.faces)return {Status::ResourceLimit};
  if(in.profile!=Profile::SingleSurfaceIlev1)return {Status::UnsupportedProfile};
  if(!in.raw_face_count)return {Status::InvalidInput};
  source::Layout physical_layout;
  const auto physical=source::MakeLayout(p,{},physical_layout);
  if(physical.status!=source_surfaces::Status::Ok)return FromPhysical(physical);
  Layout next;
  tl::util::BoundedArenaLayout output(cap.output_bytes),scratch(cap.scratch_bytes);
  const auto n=p.node_count,f=in.raw_face_count;
  if(!Output(output,f,p.solid_count,next.output)||!Output(scratch,f,p.solid_count,next.staged)||
      !scratch.Append<Vector>(n,next.points)||!scratch.Append<Key>(f,next.keys)||
      !scratch.Append<std::uint8_t>(p.solid_count,next.flags)||
      !scratch.Append<std::uint8_t>(p.solid_count,next.selected)||
      !scratch.Append<std::uint8_t>(p.solid_count,next.face_mask)||
      !scratch.Append<std::uint64_t>(p.clause.part_count,next.parts)||
      !scratch.Append<std::uint32_t>(n+1,next.solid_offsets)||
      !scratch.Append<std::uint32_t>(n+1,next.quad_offsets)||
      !scratch.Append<std::uint32_t>(n+1,next.triangle_offsets)||
      !scratch.Append<std::uint32_t>(8*p.solid_count,next.solid_rows)||
      !scratch.Append<std::uint32_t>(4*p.quad_count,next.quad_rows)||
      !scratch.Append<std::uint32_t>(3*p.triangle_count,next.triangle_rows)||
      !scratch.Append<std::uint32_t>(n,next.cursor))return {Status::ResourceLimit};
  next.forecast={output.bytes(),scratch.bytes(),f,2*f,8*p.solid_count};
  out=next;return {Status::Ok};
}
Report Admit(const Input& in,const Layout& layout,const tl::util::HostArena& out,
    const tl::util::HostArena& scratch,const Snapshot* result) noexcept {
  if(out.bytes()<layout.forecast.output_bytes||scratch.bytes()<layout.forecast.scratch_bytes)
    return {Status::ResourceLimit};
  const auto& p=in.physical;
  std::size_t positions_bytes=0;
  if(!in.source_generation||!range::VectorSpan(in.positions,p.node_count,positions_bytes)||
      !range::Span(p.solids,p.solid_count)||!range::Span(p.quads,p.quad_count)||
      !range::Span(p.triangles,p.triangle_count)||!range::Span(p.clause.part_ids,p.clause.part_count)||
      !range::Span(p.clause.solid_rows,p.clause.solid_row_count)||
      !range::Span(in.raw_faces,in.raw_face_count)||!range::Span(result,1)||
      !range::Span(static_cast<const std::byte*>(out.data()),out.bytes())||
      !range::Span(static_cast<const std::byte*>(scratch.data()),scratch.bytes()))return {Status::InvalidInput};
  if(in.coordinates!=startup::Coordinates::Native&&in.coordinates!=startup::Coordinates::Si)
    return {Status::InvalidInput};
  if(in.coordinates==startup::Coordinates::Si) {
    units_detail::Factors factors;
    if(!units_detail::Make(in.units,factors))return {Status::InvalidInput};
  }
  if((p.clause.kind==source_surfaces::ClauseKind::Parts&&(p.clause.solid_rows||p.clause.solid_row_count))||
      (p.clause.kind==source_surfaces::ClauseKind::Solids&&(p.clause.part_ids||p.clause.part_count||p.clause.reverse_shell_normals)))
    return {Status::InvalidInput};
  const Range output{out.data(),out.bytes()},work{scratch.data(),scratch.bytes()},published{result,sizeof(*result)};
  if(!Disjoint(output,work)||!Disjoint(output,published)||!Disjoint(work,published))return {Status::InvalidInput};
  const Range reads[]{{&in,sizeof(in)},{&out,sizeof(out)},{&scratch,sizeof(scratch)},
      {p.solids,p.solid_count*sizeof(source_surfaces::Solid)},{p.quads,p.quad_count*sizeof(source_surfaces::Shell)},
      {p.triangles,p.triangle_count*sizeof(source_surfaces::Shell)},
      {p.clause.part_ids,p.clause.part_count*sizeof(std::uint64_t)},
      {p.clause.solid_rows,p.clause.solid_row_count*sizeof(std::uint32_t)},
      {in.raw_faces,in.raw_face_count*sizeof(source_surfaces::Face)},{in.positions.data,positions_bytes}};
  for(const auto read:reads)
    if(!Disjoint(output,read)||!Disjoint(work,read)||!Disjoint(published,read))return {Status::InvalidInput};
  return {Status::Ok};
}
Data Construct(tl::util::HostArena& arena,const OutputLayout& p) noexcept {
  return {Values<startup::PrimaryFace>(arena,p.primary),Values<startup::PrimaryFaceIdentity>(arena,p.identities),
      Values<RawClassification>(arena,p.classifications),Values<startup::PrimaryFaceIdentity>(arena,p.raw_origins),
      Values<std::uint8_t>(arena,p.solid_flags),
      Values<std::uint32_t>(arena,p.raw_to_primary),
      Values<std::uint32_t>(arena,p.primary_to_raw)};
}
source::Work Context(tl::util::HostArena& arena,const Layout& p) noexcept {
  source::Work w;
  // Shared Prepare consumes only table validation/selection and exact CNEL
  // buffers. Its unconsumed surface-emission scratch remains null.
  w.surface_flags=Values<std::uint8_t>(arena,p.flags);
  w.selected=Values<std::uint8_t>(arena,p.selected);
  w.face_mask=Values<std::uint8_t>(arena,p.face_mask);
  w.parts=Values<std::uint64_t>(arena,p.parts);
  w.solid_offsets=Values<std::uint32_t>(arena,p.solid_offsets);
  w.quad_offsets=Values<std::uint32_t>(arena,p.quad_offsets);
  w.triangle_offsets=Values<std::uint32_t>(arena,p.triangle_offsets);
  w.solid_rows=Values<std::uint32_t>(arena,p.solid_rows);
  w.quad_rows=Values<std::uint32_t>(arena,p.quad_rows);
  w.triangle_rows=Values<std::uint32_t>(arena,p.triangle_rows);
  w.cursor=Values<std::uint32_t>(arena,p.cursor);
  return w;
}
}
