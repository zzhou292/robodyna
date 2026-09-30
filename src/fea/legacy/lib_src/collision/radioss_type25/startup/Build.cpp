// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../../RadiossType25FixedMainStartup.h"
#include "Internal.h"
#include <algorithm>
namespace tlfea::contact::radioss_type25::startup {
namespace detail {
void CopyPostGapm(const PostGapmTopology& from,Data to) noexcept {
  std::copy_n(from.primary_corners,from.primary_count,to.post_corners);
  std::copy_n(from.before_shell,from.before_shell_count,to.post_before);
  std::copy_n(from.final_support,from.main_count,to.post_support);
  *to.post_gapm=from;
  to.post_gapm->primary_corners=to.post_corners;
  to.post_gapm->before_shell=to.post_before;
  to.post_gapm->final_support=to.post_support;
}
void CopyOutput(Data from,Data to,const OutputLayout& p) noexcept {
  std::copy_n(from.mains,p.mains.count,to.mains);
  std::copy_n(from.expanded_to_primary,p.expanded_to_primary.count,to.expanded_to_primary);
  std::copy_n(from.primary_to_partner,p.primary_to_partner.count,to.primary_to_partner);
  std::copy_n(from.normals,p.normals.count,to.normals);
  std::copy_n(from.references,p.references.count,to.references);
  std::copy_n(from.normal_offsets,p.normal_offsets.count,to.normal_offsets);
  std::copy_n(from.normal_mains,p.normal_mains.count,to.normal_mains);
  if(p.primary_roles.count)std::copy_n(from.primary_roles,p.primary_roles.count,to.primary_roles);
  if(p.primary_identities.count)std::copy_n(from.primary_identities,p.primary_identities.count,to.primary_identities);
  if(p.raw_origins.count)std::copy_n(from.raw_origins,p.raw_origins.count,to.raw_origins);
  if(p.raw_origin_to_primary.count)std::copy_n(from.raw_origin_to_primary,p.raw_origin_to_primary.count,to.raw_origin_to_primary);
  if(p.post_gapm.count)CopyPostGapm(*from.post_gapm,to);
}
}
Report BuildStarter(const Input& input,Limits limits,tl::util::HostArena& output,
    tl::util::HostArena& scratch,Snapshot* published) noexcept {
  // Mixed neighbor/normal construction must not assume an all-exterior
  // IELEM_M mask. Use BuildMixedSides until post-GAPM support is authenticated.
  if(role_policy::Mixed(input.topology))return {Status::UnsupportedProfile};
  detail::Layout layout;auto report=detail::MakeLayout(input.node_count,input.primary_count,limits,layout,input.topology,input.shell_primary_count,input.raw_origin_count);
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
  if(input.topology==TopologyPolicy::ManifoldTwoSided)
    report=detail::Topology(input,staged,edges,edge_count);
  else
    report=detail::OrderedNeighbors(input,staged,points,edges,
        scratch.Construct<int>(layout.candidate_ids),scratch.Construct<double>(layout.candidate_angles),
        scratch.Construct<double>(layout.candidate_sides));
  const auto warnings=report.neighbor_warnings;
  if(report.status!=Status::Ok)return report;
  report=detail::References(input,staged,parents,tags,node_refs,references,incidence);
  report.neighbor_warnings=warnings;
  if(report.status!=Status::Ok)return report;
  // Sorted edges are no longer live. Start the neighbor-normal objects in that
  // checked arena region; this is a sequential storage reuse, not type-punned reads.
  const auto normals=4*layout.forecast.expanded_mains;
  const tl::util::ArenaRegion previous_region{layout.edges.offset,normals,normals*sizeof(StoredNormal)};
  auto* previous=scratch.Construct<StoredNormal>(previous_region);
  report=detail::StarterNormals(points,staged,input.primary_count,layout.forecast.expanded_mains,
      references,previous);
  report.neighbor_warnings=warnings;
  if(report.status!=Status::Ok)return report;
  // All rejection points precede caller output construction. Trivial value
  // construction/copies below cannot fail, allocate, or borrow source storage.
  auto committed=detail::Construct(output,layout.output);detail::CopyOutput(staged,committed,layout.output);
  Snapshot next;
  next.mains=committed.mains;next.node_count=input.node_count;next.primary_count=input.primary_count;
  next.main_count=layout.forecast.expanded_mains;
  next.expanded_to_primary=committed.expanded_to_primary;next.primary_to_partner=committed.primary_to_partner;
  next.normal_offsets=committed.normal_offsets;next.normal_mains=committed.normal_mains;
  next.normal_incidence_count=incidence;next.starter={committed.normals,committed.references,references};
  next.source_generation=input.source_generation;next.profile=input.profile;next.topology=input.topology;
  next.primary_roles=committed.primary_roles;next.primary_role_count=layout.output.primary_roles.count;
  next.primary_identities=committed.primary_identities;
  next.primary_identity_count=layout.output.primary_identities.count;
  next.shell_primary_count=role_policy::Mixed(input.topology)?input.shell_primary_count:0;
  next.raw_origins=committed.raw_origins;next.raw_origin_to_primary=committed.raw_origin_to_primary;
  next.raw_origin_count=layout.output.raw_origins.count;
  *published=next;
  Report result{Status::Ok};result.neighbor_warnings=warnings;return result;
}
} // namespace tlfea::contact::radioss_type25::startup
