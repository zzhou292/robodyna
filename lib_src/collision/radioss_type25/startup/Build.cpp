// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../../RadiossType25FixedMainStartup.h"
#include "Internal.h"
#include <algorithm>
namespace tlfea::contact::radioss_type25::startup {
namespace {
void Copy(detail::Data from,detail::Data to,const detail::OutputLayout& p) noexcept {
  std::copy_n(from.mains,p.mains.count,to.mains);
  std::copy_n(from.expanded_to_primary,p.expanded_to_primary.count,to.expanded_to_primary);
  std::copy_n(from.primary_to_partner,p.primary_to_partner.count,to.primary_to_partner);
  std::copy_n(from.normals,p.normals.count,to.normals);
  std::copy_n(from.references,p.references.count,to.references);
  std::copy_n(from.normal_offsets,p.normal_offsets.count,to.normal_offsets);
  std::copy_n(from.normal_mains,p.normal_mains.count,to.normal_mains);
}
}
Report BuildStarter(const Input& input,Limits limits,tl::util::HostArena& output,
    tl::util::HostArena& scratch,Snapshot* published) noexcept {
  detail::Layout layout;auto report=detail::MakeLayout(input.node_count,input.primary_count,limits,layout);
  if(report.status!=Status::Ok)return report;
  if(output.bytes()<layout.forecast.output_bytes || scratch.bytes()<layout.forecast.scratch_bytes)
    return {Status::ResourceLimit};
  report=detail::CheckInput(input,layout,output,scratch,published,sizeof(Snapshot));
  if(report.status!=Status::Ok)return report;
  auto staged=detail::Construct(scratch,layout.output);
  auto* points=scratch.Construct<Vector>(layout.points);
  auto* ids=scratch.Construct<detail::Identity>(layout.identities);
  auto* face_keys=scratch.Construct<detail::FaceKey>(layout.face_keys);
  auto* edges=scratch.Construct<detail::Edge>(layout.edges);
  auto* parents=scratch.Construct<int>(layout.parents);
  auto* tags=scratch.Construct<int>(layout.tags);
  auto* node_refs=scratch.Construct<std::uint32_t>(layout.node_references);
  report=detail::Expand(input,staged,points,ids,face_keys);
  if(report.status!=Status::Ok)return report;
  std::size_t edge_count=0,references=0,incidence=0;
  report=detail::Topology(input,staged,edges,edge_count);
  if(report.status!=Status::Ok)return report;
  report=detail::References(input,staged,parents,tags,node_refs,references,incidence);
  if(report.status!=Status::Ok)return report;
  // Sorted edges are no longer live. Start the neighbor-normal objects in that
  // checked arena region; this is a sequential storage reuse, not type-punned reads.
  const auto normals=4*layout.forecast.expanded_mains;
  const tl::util::ArenaRegion previous_region{layout.edges.offset,normals,normals*sizeof(StoredNormal)};
  auto* previous=scratch.Construct<StoredNormal>(previous_region);
  report=detail::StarterNormals(points,staged,input.primary_count,layout.forecast.expanded_mains,
      references,previous);
  if(report.status!=Status::Ok)return report;
  // All rejection points precede caller output construction. Trivial value
  // construction/copies below cannot fail, allocate, or borrow source storage.
  auto committed=detail::Construct(output,layout.output);Copy(staged,committed,layout.output);
  Snapshot next;
  next.mains=committed.mains;next.node_count=input.node_count;next.primary_count=input.primary_count;
  next.main_count=layout.forecast.expanded_mains;
  next.expanded_to_primary=committed.expanded_to_primary;next.primary_to_partner=committed.primary_to_partner;
  next.normal_offsets=committed.normal_offsets;next.normal_mains=committed.normal_mains;
  next.normal_incidence_count=incidence;next.starter={committed.normals,committed.references,references};
  next.source_generation=input.source_generation;*published=next;
  return {Status::Ok};
}
} // namespace tlfea::contact::radioss_type25::startup
