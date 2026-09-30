// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
#include <algorithm>
namespace tlfea::contact::radioss_type25::surface_interface {
Report Preflight(const Input& in,Limits limits,Forecast& out) noexcept {
  detail::Layout layout;
  auto report=detail::MakeLayout(in,limits,layout);
  if(report.status!=Status::Ok)return report;
  using source_nodal::detail::Range;
  using source_nodal::detail::Disjoint;
  if(!search::detail::Span(&out,1))return {Status::InvalidInput};
  const auto& p=in.physical;
  std::size_t positions_bytes=0;
  if(!search::detail::VectorSpan(in.positions,p.node_count,positions_bytes))return {Status::InvalidInput};
  const Range reads[]{{&in,sizeof(in)},{p.solids,p.solid_count*sizeof(source_surfaces::Solid)},
      {p.quads,p.quad_count*sizeof(source_surfaces::Shell)},{p.triangles,p.triangle_count*sizeof(source_surfaces::Shell)},
      {p.clause.part_ids,p.clause.part_count*sizeof(std::uint64_t)},
      {p.clause.solid_rows,p.clause.solid_row_count*sizeof(std::uint32_t)},
      {in.raw_faces,in.raw_face_count*sizeof(source_surfaces::Face)},{in.positions.data,positions_bytes}};
  for(const auto read:reads)if(!Disjoint({&out,sizeof(out)},read))return {Status::InvalidInput};
  out=layout.forecast;return {Status::Ok};
}
Report Build(const Input& in,Limits limits,tl::util::HostArena& output,
    tl::util::HostArena& scratch,Snapshot* published) noexcept {
  detail::Layout layout;
  auto report=detail::MakeLayout(in,limits,layout);
  if(report.status!=Status::Ok)return report;
  report=detail::Admit(in,layout,output,scratch,published);
  if(report.status!=Status::Ok)return report;
  auto staged=detail::Construct(scratch,layout.staged);
  auto context=detail::Context(scratch,layout);
  auto* points=scratch.Construct<Vector>(layout.points);
  auto* keys=scratch.Construct<detail::Key>(layout.keys);
  report=detail::Prepare(in,context,points);
  if(report.status!=Status::Ok)return report;
  report=detail::Classify(in,context,points,staged);
  if(report.status!=Status::Ok)return report;
  std::size_t primaries=0,shells=0;
  report=detail::Filter(in,staged,keys,primaries,shells);
  if(report.status!=Status::Ok)return report;
  auto committed=detail::Construct(output,layout.output);
  std::copy_n(staged.primary,primaries,committed.primary);
  std::copy_n(staged.identities,primaries,committed.identities);
  std::copy_n(staged.primary_to_raw,primaries,committed.primary_to_raw);
  std::copy_n(staged.classifications,in.raw_face_count,committed.classifications);
  std::copy_n(staged.raw_to_primary,in.raw_face_count,committed.raw_to_primary);
  std::copy_n(staged.raw_origins,in.raw_face_count,committed.raw_origins);
  if(in.physical.solid_count)
    std::copy_n(staged.solid_flags,in.physical.solid_count,committed.solid_flags);
  Snapshot next;
  next.primary=committed.primary;next.identities=committed.identities;
  next.primary_count=primaries;next.shell_primary_count=shells;next.main_count=primaries+shells;
  next.classifications=committed.classifications;next.raw_to_primary=committed.raw_to_primary;
  next.raw_origins=committed.raw_origins;
  next.surface_solid_flags=committed.solid_flags;next.physical_solid_count=in.physical.solid_count;
  next.primary_to_raw=committed.primary_to_raw;next.raw_face_count=in.raw_face_count;
  next.source_generation=in.source_generation;
  *published=next;return {Status::Ok};
}
}
