// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Layout.h"
#include "Values.h"
#include "../search/Ranges.h"
#include <climits>
namespace tlfea::contact::radioss_type25::source_shells::detail {
Report Prepare(const Input& input, Limits limits, Layout& output) noexcept {
  const Limits hard;
  if (!limits.nodes || limits.nodes>hard.nodes || !limits.shells || limits.shells>hard.shells ||
      limits.primaries>hard.primaries || limits.secondaries>hard.secondaries ||
      !limits.scratch_bytes || limits.scratch_bytes>hard.scratch_bytes ||
      input.node_count>limits.nodes || input.shell_count>limits.shells ||
      input.primary_count>limits.primaries || input.secondary_count>limits.secondaries)
    return {Status::ResourceLimit};
  if (!Supported(input.profile)) return {Status::UnsupportedProfile};
  namespace c=coefficient_detail;
  const auto& profile=input.profile;
  if (!input.node_count || !input.shell_count ||
      !c::Nonnegative(profile.stiffness_scale) || !c::Nonnegative(profile.gap_scale) ||
      !c::Nonnegative(profile.maximum_secondary_gap) || !c::Nonnegative(profile.maximum_main_gap) ||
      !search::detail::Span(input.shells,input.shell_count) ||
      !search::detail::Span(input.primary_shells,input.primary_count) ||
      !search::detail::Span(input.secondary,input.secondary_count)) return {Status::InvalidInput};
  Layout next;
  tl::util::BoundedArenaLayout arena(limits.scratch_bytes);
  if (!arena.Append<NodeFields>(input.node_count,next.nodes) ||
      !arena.Append<double>(input.primary_count,next.primaries) ||
      !arena.Append<SecondaryFields>(input.secondary_count,next.secondary)) return {Status::ResourceLimit};
  next.forecast.output_bytes=arena.bytes();
  if (!arena.Append<unsigned char>(input.shell_count,next.selected_shells)) return {Status::ResourceLimit};
  next.forecast.scratch_bytes=arena.bytes();
  // Native SPMD_MSIN processes Q4 then T3, sorting original element IDs within
  // each family. The explicit source roster retains that reduction order.
  std::size_t q4_count=0;
  unsigned previous_family=0;
  std::uint64_t previous_id=0;
  for (std::size_t i=0;i<input.shell_count;++i) {
    const auto& shell=input.shells[i];
    const auto slots=Slots(shell.layout);
    const unsigned family=slots==4?1:(slots==3?2:0);
    Report bad{Status::InvalidInput};bad.physical_shell=i;
    if (!family || !shell.source_element_id || family<previous_family ||
        (family==previous_family && shell.source_element_id<=previous_id) ||
        !c::Nonnegative(shell.young) || !c::Nonnegative(shell.structural_thickness) ||
        !c::Nonnegative(shell.element_thickness) || !c::Nonnegative(shell.property_thickness) ||
        !c::Nonnegative(shell.part_contact_thickness)) return bad;
    if (slots==3 && shell.nodes[3]!=shell.nodes[2]) return bad;
    for (unsigned j=0;j<slots;++j) {
      if (shell.nodes[j]>=input.node_count) return bad;
      for (unsigned k=0;k<j;++k) if (shell.nodes[j]==shell.nodes[k]) return bad;
    }
    if (family==1) ++q4_count;
    else {
      std::size_t lower=0,upper=q4_count;
      while (lower<upper) {
        const auto middle=lower+(upper-lower)/2;
        if (input.shells[middle].source_element_id<shell.source_element_id) lower=middle+1;
        else upper=middle;
      }
      if (lower<q4_count && input.shells[lower].source_element_id==shell.source_element_id) return bad;
    }
    previous_family=family;previous_id=shell.source_element_id;
  }
  for (std::size_t i=0;i<input.primary_count;++i) {
    if (input.primary_shells[i]>=input.shell_count) {
      Report bad{Status::InvalidInput};bad.primary=i;return bad;
    }
  }
  for (std::size_t i=0;i<input.secondary_count;++i) {
    if (input.secondary[i].node>=input.node_count || !c::Finite(input.secondary[i].existing_coefficient)) {
      Report bad{Status::InvalidInput};bad.secondary=i;return bad;
    }
  }
  output=next;return {Status::Ok};
}

bool SeparateStorage(const Input& input, const Layout& layout, void* scratch,
    std::size_t bytes, Output out) noexcept {
  if (bytes<layout.forecast.scratch_bytes || !scratch ||
      reinterpret_cast<std::uintptr_t>(scratch)%alignof(std::max_align_t) ||
      bytes>UINTPTR_MAX-reinterpret_cast<std::uintptr_t>(scratch) ||
      out.node_count!=input.node_count || out.primary_count!=input.primary_count ||
      out.secondary_count!=input.secondary_count ||
      !search::detail::Span(out.nodes,out.node_count) ||
      !search::detail::Span(out.primary_stiffness,out.primary_count) ||
      !search::detail::Span(out.secondary,out.secondary_count)) return false;
  struct Range { const void* data; std::size_t bytes; };
  const Range read[]{ {&input,sizeof(input)}, {input.shells,input.shell_count*sizeof(PhysicalShell)},
      {input.primary_shells,input.primary_count*sizeof(std::uint32_t)},
      {input.secondary,input.secondary_count*sizeof(Secondary)} };
  const Range write[]{ {scratch,bytes}, {out.nodes,out.node_count*sizeof(NodeFields)},
      {out.primary_stiffness,out.primary_count*sizeof(double)},
      {out.secondary,out.secondary_count*sizeof(SecondaryFields)} };
  auto separate=[](Range a,Range b) {
    if (!a.bytes || !b.bytes) return true;
    const auto first=reinterpret_cast<std::uintptr_t>(a.data);
    const auto second=reinterpret_cast<std::uintptr_t>(b.data);
    return a.data && b.data && a.bytes<=UINTPTR_MAX-first && b.bytes<=UINTPTR_MAX-second &&
        (first+a.bytes<=second || second+b.bytes<=first);
  };
  for (unsigned i=0;i<4;++i) {
    for (const auto& source:read) if (!separate(write[i],source)) return false;
    for (unsigned j=0;j<i;++j) if (!separate(write[i],write[j])) return false;
  }
  return true;
}
} // namespace tlfea::contact::radioss_type25::source_shells::detail
