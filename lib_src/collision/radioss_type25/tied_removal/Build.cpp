// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
#include <algorithm>
namespace tlfea::contact::radioss_type25::tied_removal {
Report Build(const Input& in,Limits limits,tl::util::HostArena& output,
    tl::util::HostArena& scratch,Snapshot* published) noexcept {
  if(!search::detail::Span(published,1))return {Status::InvalidInput};
  detail::Layout layout;auto report=detail::Plan(in,limits,layout);
  if(report.status!=Status::Ok)return report;
  if(output.bytes()<layout.forecast.output_bytes||scratch.bytes()<layout.forecast.scratch_bytes)
    return {Status::ResourceLimit};
  report=detail::Admit(in,limits,layout,output,scratch,published);
  if(report.status!=Status::Ok)return report;
  auto staged=detail::Construct(scratch,layout.staged);auto work=detail::ConstructWork(scratch,layout);
  report=detail::Prepare(in,work);
  if(report.status!=Status::Ok)return report;
  report=detail::Evaluate(in,limits,layout,work,staged);
  if(report.status!=Status::Ok)return report;
  const auto& old=in.geometric.geometry;
  auto committed=detail::Construct(output,layout.output);
  std::copy_n(old.primary_extent,old.primary_count,committed.extent);
  std::copy_n(old.initial_contact,old.secondary_count,committed.contact);
  std::copy_n(staged.main_offsets,old.main_count+1,committed.main_offsets);
  std::copy_n(staged.secondary_offsets,old.secondary_count+1,committed.secondary_offsets);
  if(report.required_removals) {
    std::copy_n(staged.nodes,report.required_removals,committed.nodes);
    std::copy_n(staged.mains,report.required_removals,committed.mains);
  }
  std::copy_n(staged.history,old.secondary_count,committed.history);
  Snapshot next;next.search=old;
  next.search.primary_extent=committed.extent;next.search.initial_contact=committed.contact;
  next.search.main_offsets=committed.main_offsets;next.search.secondary_offsets=committed.secondary_offsets;
  next.search.removed_nodes=report.required_removals?committed.nodes:nullptr;
  next.search.removed_mains=report.required_removals?committed.mains:nullptr;
  next.search.removal_count=report.required_removals;
  next.history=committed.history;next.history_count=old.secondary_count;
  next.added_removals=report.added_removals;next.reset_rows=report.reset_rows;
  next.native_removal_extent=in.native_removal_extent+report.added_removals;
  *published=next;return report;
}
} // namespace tlfea::contact::radioss_type25::tied_removal
