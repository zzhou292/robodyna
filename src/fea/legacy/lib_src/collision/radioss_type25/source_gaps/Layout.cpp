// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Layout.h"
#include <iterator>
#include <type_traits>
#include "Values.h"
#include "../search/Ranges.h"
#include "../source_nodal/HostRanges.h"
namespace tlfea::contact::radioss_type25::source_gaps::detail {
Report Prepare(const Input& in, Limits cap, Layout& output) noexcept {
  const Limits hard;
  const std::size_t limits[]{cap.nodes,cap.shells,cap.lines,cap.springs,cap.mains,
    cap.secondaries,cap.main_nodes,cap.scratch_bytes,cap.output_bytes};
  const std::size_t maxima[]{hard.nodes,hard.shells,hard.lines,hard.springs,hard.mains,
    hard.secondaries,hard.main_nodes,hard.scratch_bytes,hard.output_bytes};
  for(unsigned i=0;i<std::size(limits);++i)
    if(!limits[i] || limits[i]>maxima[i]) return {Status::ResourceLimit};
  if(in.node_count>cap.nodes || in.shell_count>cap.shells || in.truss_count>cap.lines ||
      in.beam_count>cap.lines-in.truss_count || in.spring_count>cap.springs ||
      in.main_count>cap.mains || in.secondary_count>cap.secondaries ||
      in.main_node_count>cap.main_nodes) return {Status::ResourceLimit};
  if(!Supported(in.profile)) return {Status::UnsupportedProfile};
  namespace c=coefficient_detail;
  if(!in.node_count || in.primary_count>in.main_count ||
      !c::Nonnegative(in.profile.scale) || !c::Nonnegative(in.profile.maximum_main) ||
      !c::Nonnegative(in.profile.maximum_secondary) ||
      !search::detail::Span(in.shells,in.shell_count) ||
      !search::detail::Span(in.trusses,in.truss_count) || !search::detail::Span(in.beams,in.beam_count) ||
      !search::detail::Span(in.springs,in.spring_count) || !search::detail::Span(in.mains,in.main_count) ||
      !search::detail::Span(in.secondary_nodes,in.secondary_count) ||
      !search::detail::Span(in.main_nodes,in.main_node_count)) return {Status::InvalidInput};
  Layout next;
  tl::util::BoundedArenaLayout output_layout(cap.output_bytes);
  if(!output_layout.Append<double>(in.secondary_count,next.secondary) ||
      !output_layout.Append<double>(in.main_node_count,next.main_nodes) ||
      !output_layout.Append<MainGapFields>(in.main_count,next.mains)) return {Status::ResourceLimit};
  next.forecast.output_bytes=output_layout.bytes();
  tl::util::BoundedArenaLayout arena(cap.scratch_bytes);
  if(!arena.Append<double>(in.secondary_count,next.secondary) ||
      !arena.Append<double>(in.main_node_count,next.main_nodes) ||
      !arena.Append<MainGapFields>(in.main_count,next.mains) ||
      !arena.Append<double>(in.node_count,next.secondary_work) ||
      !arena.Append<double>(in.node_count,next.main_work) ||
      !arena.Append<unsigned char>(in.node_count,next.tags)) return {Status::ResourceLimit};
  next.forecast.scratch_bytes=arena.bytes();
  output=next;
  return {Status::Ok};
}
bool Disjoint(const Input& in,const Layout& plan,void* scratch,std::size_t bytes,Output out) noexcept {
  if(!scratch || bytes<plan.forecast.scratch_bytes ||
      reinterpret_cast<std::uintptr_t>(scratch)%alignof(std::max_align_t) ||
      bytes>UINTPTR_MAX-reinterpret_cast<std::uintptr_t>(scratch) ||
      out.secondary_count!=in.secondary_count || out.main_node_count!=in.main_node_count ||
      out.main_count!=in.main_count || !search::detail::Span(out.secondary,out.secondary_count) ||
      !search::detail::Span(out.main_nodes,out.main_node_count) || !search::detail::Span(out.mains,out.main_count)) return false;
  using source_nodal::detail::Range;
  const Range read[]{{&in,sizeof(in)},{in.shells,in.shell_count*sizeof(PhysicalShell)},
    {in.trusses,in.truss_count*sizeof(Line)},{in.beams,in.beam_count*sizeof(Line)},
    {in.springs,in.spring_count*sizeof(Spring)},{in.mains,in.main_count*sizeof(startup::Main)},
    {in.secondary_nodes,in.secondary_count*sizeof(std::uint32_t)},
    {in.main_nodes,in.main_node_count*sizeof(std::uint32_t)}};
  const Range write[]{{scratch,bytes},{out.secondary,out.secondary_count*sizeof(double)},
    {out.main_nodes,out.main_node_count*sizeof(double)},{out.mains,out.main_count*sizeof(MainGapFields)}};
  for(unsigned i=0;i<std::size(write);++i) {
    for(const auto& r:read) if(!source_nodal::detail::Disjoint(write[i],r)) return false;
    for(unsigned j=0;j<i;++j) if(!source_nodal::detail::Disjoint(write[i],write[j])) return false;
  }
  return true;
}
} // namespace tlfea::contact::radioss_type25::source_gaps::detail
