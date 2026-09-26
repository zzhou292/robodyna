// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
namespace tlfea::contact::radioss_type25::source_surfaces::detail {
Report MakeLayout(const Input& in,Limits cap,Layout& out) noexcept {
  const Limits hard;
  if(!cap.nodes||cap.nodes>hard.nodes||cap.solids>hard.solids||cap.shells>hard.shells||
      cap.parts>hard.parts||cap.faces>hard.faces||!cap.output_bytes||cap.output_bytes>hard.output_bytes||
      !cap.scratch_bytes||cap.scratch_bytes>hard.scratch_bytes||in.node_count>cap.nodes||
      in.solid_count>cap.solids||in.quad_count>cap.shells||in.triangle_count>cap.shells-in.quad_count||
      in.clause.part_count>cap.parts||in.clause.solid_row_count>in.solid_count)
    return {Status::ResourceLimit};
  if(in.phase!=ReaderPhase::BeforeGroupingAndInitia ||
      (in.clause.kind!=ClauseKind::Parts&&in.clause.kind!=ClauseKind::Solids) ||
      (in.clause.mode!=SurfaceMode::Exterior&&in.clause.mode!=SurfaceMode::ExteriorShellEdges&&in.clause.mode!=SurfaceMode::All))
    return {Status::UnsupportedProfile};
  if(!in.node_count||(!in.solid_count&&!in.quad_count&&!in.triangle_count))return {Status::InvalidInput};
  Layout next;
  next.forecast.maximum_faces=6*in.solid_count+in.quad_count+in.triangle_count;
  next.forecast.solid_incidence=8*in.solid_count;
  next.forecast.quad_incidence=4*in.quad_count;
  next.forecast.triangle_incidence=3*in.triangle_count;
  if(next.forecast.maximum_faces>cap.faces)return {Status::ResourceLimit};
  tl::util::BoundedArenaLayout output(cap.output_bytes),scratch(cap.scratch_bytes);
  if(!output.Append<Face>(next.forecast.maximum_faces,next.faces)||
      !output.Append<std::uint8_t>(in.solid_count,next.flags)||
      !scratch.Append<Face>(next.forecast.maximum_faces,next.staged_faces)||
      !scratch.Append<std::uint8_t>(in.solid_count,next.staged_flags)||
      !scratch.Append<std::uint8_t>(in.solid_count,next.selected)||
      !scratch.Append<std::uint8_t>(in.solid_count,next.face_mask)||
      !scratch.Append<std::uint64_t>(in.clause.part_count,next.parts)||
      !scratch.Append<std::uint32_t>(in.node_count+1,next.solid_offsets)||
      !scratch.Append<std::uint32_t>(in.node_count+1,next.quad_offsets)||
      !scratch.Append<std::uint32_t>(in.node_count+1,next.triangle_offsets)||
      !scratch.Append<std::uint32_t>(next.forecast.solid_incidence,next.solid_rows)||
      !scratch.Append<std::uint32_t>(next.forecast.quad_incidence,next.quad_rows)||
      !scratch.Append<std::uint32_t>(next.forecast.triangle_incidence,next.triangle_rows)||
      !scratch.Append<std::uint32_t>(in.node_count,next.cursor))return {Status::ResourceLimit};
  next.forecast.output_bytes=output.bytes();next.forecast.scratch_bytes=scratch.bytes();
  out=next;return {Status::Ok};
}
Report Admit(const Input& in,const Layout& layout,const tl::util::HostArena& out,
    const tl::util::HostArena& scratch,const Snapshot* result) noexcept {
  namespace range=search::detail;
  if(out.bytes()<layout.forecast.output_bytes||scratch.bytes()<layout.forecast.scratch_bytes)
    return {Status::ResourceLimit};
  if(!range::Span(in.solids,in.solid_count)||!range::Span(in.quads,in.quad_count)||
      !range::Span(in.triangles,in.triangle_count)||!range::Span(in.clause.part_ids,in.clause.part_count)||
      !range::Span(in.clause.solid_rows,in.clause.solid_row_count)||!range::Span(result,1)||
      !range::Span(static_cast<const std::byte*>(out.data()),out.bytes())||
      !range::Span(static_cast<const std::byte*>(scratch.data()),scratch.bytes()))return {Status::InvalidInput};
  if((in.clause.kind==ClauseKind::Parts&&(in.clause.solid_rows||in.clause.solid_row_count))||
      (in.clause.kind==ClauseKind::Solids&&(in.clause.part_ids||in.clause.part_count||in.clause.reverse_shell_normals)))
    return {Status::InvalidInput};
  using source_nodal::detail::Range;using source_nodal::detail::Disjoint;
  const Range output{out.data(),out.bytes()},work{scratch.data(),scratch.bytes()},descriptor{result,sizeof(*result)};
  if(!Disjoint(output,work)||!Disjoint(output,descriptor)||!Disjoint(work,descriptor))return {Status::InvalidInput};
  const Range reads[]{{&in,sizeof(in)},{&out,sizeof(out)},{&scratch,sizeof(scratch)},
      {in.solids,in.solid_count*sizeof(Solid)},
      {in.quads,in.quad_count*sizeof(Shell)},{in.triangles,in.triangle_count*sizeof(Shell)},
      {in.clause.part_ids,in.clause.part_count*sizeof(std::uint64_t)},
      {in.clause.solid_rows,in.clause.solid_row_count*sizeof(std::uint32_t)}};
  for(const auto read:reads)
    if(!Disjoint(output,read)||!Disjoint(work,read)||!Disjoint(descriptor,read))return {Status::InvalidInput};
  return {Status::Ok};
}
Work Borrow(tl::util::HostArena& arena,const Layout& p) noexcept {
  Work w;
  w.faces=arena.Construct<Face>(p.staged_faces);
  w.surface_flags=p.staged_flags.count?arena.Construct<std::uint8_t>(p.staged_flags):nullptr;
  w.selected=p.selected.count?arena.Construct<std::uint8_t>(p.selected):nullptr;
  w.face_mask=p.face_mask.count?arena.Construct<std::uint8_t>(p.face_mask):nullptr;
  w.parts=p.parts.count?arena.Construct<std::uint64_t>(p.parts):nullptr;
  w.solid_offsets=arena.Construct<std::uint32_t>(p.solid_offsets);
  w.quad_offsets=arena.Construct<std::uint32_t>(p.quad_offsets);
  w.triangle_offsets=arena.Construct<std::uint32_t>(p.triangle_offsets);
  w.solid_rows=p.solid_rows.count?arena.Construct<std::uint32_t>(p.solid_rows):nullptr;
  w.quad_rows=p.quad_rows.count?arena.Construct<std::uint32_t>(p.quad_rows):nullptr;
  w.triangle_rows=p.triangle_rows.count?arena.Construct<std::uint32_t>(p.triangle_rows):nullptr;
  w.cursor=arena.Construct<std::uint32_t>(p.cursor);return w;
}
}
