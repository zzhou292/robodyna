// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
namespace tlfea::contact::radioss_type25::source_surfaces {
Report Preflight(const Input& input,Limits limits,Forecast& output) noexcept {
  detail::Layout layout;
  const auto report=detail::MakeLayout(input,limits,layout);
  if(report.status!=Status::Ok)return report;
  using source_nodal::detail::Range;
  using source_nodal::detail::Disjoint;
  const Range destination{&output,sizeof(output)};
  const Range reads[]{
      {&input,sizeof(input)},
      {input.solids,input.solid_count*sizeof(Solid)},
      {input.quads,input.quad_count*sizeof(Shell)},
      {input.triangles,input.triangle_count*sizeof(Shell)},
      {input.clause.part_ids,input.clause.part_count*sizeof(std::uint64_t)},
      {input.clause.solid_rows,input.clause.solid_row_count*sizeof(std::uint32_t)}};
  if(!search::detail::Span(&output,1))return {Status::InvalidInput};
  for(const auto range:reads)if(!Disjoint(destination,range))return {Status::InvalidInput};
  output=layout.forecast;
  return report;
}
Report Build(const Input& input,Limits limits,tl::util::HostArena& output,
    tl::util::HostArena& scratch,Snapshot* result) noexcept {
  detail::Layout layout;auto report=detail::MakeLayout(input,limits,layout);
  if(report.status!=Status::Ok)return report;
  report=detail::Admit(input,layout,output,scratch,result);if(report.status!=Status::Ok)return report;
  auto work=detail::Borrow(scratch,layout);Counts counts;
  report=detail::Prepare(input,work,counts);if(report.status!=Status::Ok)return report;
  std::size_t count=0;report=detail::Extract(input,work,counts,count);if(report.status!=Status::Ok)return report;
  auto* faces=output.Construct<Face>(layout.faces);
  auto* flags=input.solid_count?output.Construct<std::uint8_t>(layout.flags):nullptr;
  if(count)std::memcpy(faces,work.faces,count*sizeof(Face));
  if(input.solid_count)std::memcpy(flags,work.surface_flags,input.solid_count);
  Snapshot next;next.faces=count?faces:nullptr;next.face_count=count;
  next.surface_solid_flags=flags;next.solid_count=input.solid_count;next.counts=counts;
  *result=next;return report;
}
}
