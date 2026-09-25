// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../../RadiossType25SearchStartup.h"
#include "Internal.h"
namespace tlfea::contact::radioss_type25::search_startup {
Report Build(const Input& in,Limits limits,tl::util::HostArena& output,
    tl::util::HostArena& scratch,Snapshot* published) noexcept {
  detail::Layout layout;
  auto report=detail::MakeLayout(in.mesh.node_count,in.mesh.primary_count,in.secondary_count,limits,layout);
  if(report.status!=Status::Ok)return report;
  if(output.bytes()<layout.forecast.output_bytes || scratch.bytes()<layout.forecast.scratch_bytes)
    return {Status::ResourceLimit};
  report=detail::Admit(in,layout,limits,output,scratch,published);
  if(report.status!=Status::Ok)return report;
  auto staged=detail::Construct(scratch,layout.output);
  auto work=detail::ConstructWork(scratch,layout);
  double secondary_gap=0,gap=0;
  report=detail::Prepare(in,work,secondary_gap,gap);
  if(report.status!=Status::Ok)return report;
  Snapshot next;
  auto status=ResolveMultiplier(in.contributors.physical_nodes,&next.multiplier);
  if(status!=Status::Ok)return {status};
  report=detail::Margin(in,work.points,next.multiplier,gap,limits,next.mean_length,next.margin);
  if(report.status!=Status::Ok)return report;
  report=detail::Extent(in,work.points,staged.extent,next.maximum_extent);
  if(report.status!=Status::Ok)return report;
  report=detail::Removals(in,limits,work,staged,layout.forecast.removal_capacity,secondary_gap);
  if(report.status!=Status::Ok)return report;
  // Source allocation initializes ICONT_I=0; selected INACTI5 PWR3 never
  // writes it. This does not replace native initial penetration/history work.
  next.removal_count=report.required_removals;
  const auto committed=detail::Construct(output,layout.output);
  std::copy_n(staged.extent,in.mesh.primary_count,committed.extent);
  std::copy_n(staged.main_offsets,in.main_count+1,committed.main_offsets);
  std::copy_n(staged.secondary_offsets,in.secondary_count+1,committed.secondary_offsets);
  std::copy_n(staged.removed_nodes,next.removal_count,committed.removed_nodes);
  std::copy_n(staged.removed_mains,next.removal_count,committed.removed_mains);
  std::copy_n(staged.contact,in.secondary_count,committed.contact);
  next.primary_extent=committed.extent;next.primary_count=in.mesh.primary_count;
  next.main_offsets=committed.main_offsets;next.secondary_offsets=committed.secondary_offsets;
  next.removed_nodes=next.removal_count?committed.removed_nodes:nullptr;
  next.removed_mains=next.removal_count?committed.removed_mains:nullptr;
  next.initial_contact=committed.contact;
  next.main_count=in.main_count;next.secondary_count=in.secondary_count;
  next.source_generation=in.mesh.source_generation;
  *published=next;return report;
}
} // namespace tlfea::contact::radioss_type25::search_startup
