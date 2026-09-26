// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../../RadiossType25FixedMainStartup.h"
#include "Internal.h"
#include "../search/Ranges.h"
#include <algorithm>
namespace tlfea::contact::radioss_type25::startup {
namespace {
namespace d=detail;
namespace range=search::detail;
bool Same(const PrimaryFaceIdentity& a,const PrimaryFaceIdentity& b) noexcept {
  return a.kind==b.kind && a.physical_parent_id==b.physical_parent_id && a.local_face==b.local_face &&
      a.origin==b.origin && a.origin_count==b.origin_count;
}
Report Descriptors(const Input& input,const MixedSidesSnapshot& sides,const PostGapmTopology& post) noexcept {
  const auto p=input.primary_count,g=p+input.shell_primary_count,raw=input.raw_origin_count;
  if(input.profile!=Profile::MixedSurface || input.topology!=TopologyPolicy::NativeMixedSurface)
    return {Status::UnsupportedProfile};
  if(sides.primary_count!=p || sides.shell_primary_count!=input.shell_primary_count || sides.main_count!=g ||
      sides.node_count!=input.node_count || sides.raw_origin_count!=raw || !input.source_generation ||
      sides.source_generation!=input.source_generation || post.source_generation!=input.source_generation ||
      post.phase!=PostGapmPhase::FinalizedBeforeNeighbors || post.primary_count!=p || post.before_shell_count!=p ||
      post.main_count!=g || !range::Span(sides.mains,g) || !range::Span(sides.primary_to_partner,p) ||
      !range::Span(sides.expanded_to_primary,g) || !range::Span(sides.primary_roles,p) ||
      !range::Span(sides.primary_identities,p) || !range::Span(sides.raw_origins,raw) ||
      !range::Span(sides.raw_origin_to_primary,raw) || !range::Span(post.primary_corners,p) ||
      !range::Span(post.before_shell,p) || !range::Span(post.final_support,g))return {Status::InvalidInput};
  if((post.incoming_solid_erosion!=SolidErosion::Disabled && post.incoming_solid_erosion!=SolidErosion::Enabled) ||
      (post.final_solid_erosion!=SolidErosion::Disabled && post.final_solid_erosion!=SolidErosion::Enabled))
    return {Status::UnsupportedProfile};
  std::size_t internal=0;
  for(std::size_t i=0;i<p;++i) {
    const auto& before=post.before_shell[i];const auto& support=post.final_support[i];
    if(before.unique_match_count>2)return {Status::UnsupportedProfile,i};
    if((!before.unique_match_count && (before.first_solid_source_id || before.second_solid_source_id)) ||
        (before.unique_match_count && !before.first_solid_source_id) ||
        (before.unique_match_count<2 && before.second_solid_source_id) ||
        (before.second_solid_source_id && before.first_solid_source_id==before.second_solid_source_id))
      return {Status::InvalidInput,i};
    internal+=before.second_solid_source_id!=0;
    const bool triangle=sides.mains[i].nodes[2]==sides.mains[i].nodes[3];
    const unsigned identity[]{0,1,2,triangle?2u:3u},q[]{3,2,1,0},t[]{1,0,2,2};
    bool unchanged=true,reversed=true;
    for(unsigned k=0;k<4;++k) {
      unchanged=unchanged && post.primary_corners[i].source_corner[k]==identity[k];
      reversed=reversed && post.primary_corners[i].source_corner[k]==(triangle?t[k]:q[k]);
    }
    if(!unchanged && !(reversed && before.first_solid_source_id && !before.second_solid_source_id))
      return {Status::UnsupportedTopology,i};
    if(!support.first.source_element_id)return {Status::InvalidInput,i};
    if(support.first.kind==PhysicalSupportKind::EightSlotSolid) {
      if(support.first.source_element_id!=before.first_solid_source_id ||
          support.second_solid_source_id!=before.second_solid_source_id)return {Status::InvalidInput,i};
    } else if((support.first.kind!=PhysicalSupportKind::ShellQuad && support.first.kind!=PhysicalSupportKind::ShellTriangle) ||
        support.second_solid_source_id)return {Status::InvalidInput,i};
    const auto partner=sides.primary_to_partner[i];
    if(sides.primary_identities[i].kind==PrimaryFaceKind::Shell) {
      if(partner<=p || partner>g || support.first.kind==PhysicalSupportKind::EightSlotSolid)
        return {Status::UnsupportedTopology,i};
      const auto& other=post.final_support[partner-1];
      if(other.first.kind!=support.first.kind || other.first.source_element_id!=support.first.source_element_id ||
          other.second_solid_source_id)return {Status::InvalidInput,i};
    } else if(sides.primary_identities[i].kind!=PrimaryFaceKind::Solid || partner)
      return {Status::UnsupportedTopology,i};
  }
  if(internal!=post.pre_shell_internal_count ||
      post.final_solid_erosion!=(internal?post.incoming_solid_erosion:SolidErosion::Disabled))
    return {Status::InvalidInput};
  return {Status::Ok};
}
bool SourceDisjoint(const MixedSidesSnapshot& s,const PostGapmTopology& p,
    const void* target,std::size_t bytes) noexcept {
  struct Range {const void* pointer;std::size_t bytes;};
  const Range reads[]{{&s,sizeof(s)},{&p,sizeof(p)},
    {s.mains,s.main_count*sizeof(Main)},{s.expanded_to_primary,s.main_count*sizeof(std::uint32_t)},
    {s.primary_to_partner,s.primary_count*sizeof(std::uint32_t)},
    {s.primary_roles,s.primary_count*sizeof(ShellSideRole)},
    {s.primary_identities,s.primary_count*sizeof(PrimaryFaceIdentity)},
    {s.raw_origins,s.raw_origin_count*sizeof(PrimaryFaceIdentity)},
    {s.raw_origin_to_primary,s.raw_origin_count*sizeof(std::uint32_t)},
    {p.primary_corners,p.primary_count*sizeof(PrimaryCornerPermutation)},
    {p.before_shell,p.before_shell_count*sizeof(PreShellSolidSupport)},
    {p.final_support,p.main_count*sizeof(PostGapmMainSupport)}};
  for(const auto& value:reads)if(!d::Disjoint(target,bytes,value.pointer,value.bytes))return false;
  return true;
}
Report SameSides(const Input& input,const MixedSidesSnapshot& sides,d::Data expected) noexcept {
  const auto p=input.primary_count,g=sides.main_count;
  for(std::size_t i=0;i<g;++i) {
    const auto& a=sides.mains[i];const auto& b=expected.mains[i];
    if(a.source_id!=b.source_id || a.global_id!=b.global_id || a.segment_type!=b.segment_type ||
        sides.expanded_to_primary[i]!=expected.expanded_to_primary[i])return {Status::InvalidInput,i<p?i:SIZE_MAX};
    for(unsigned k=0;k<4;++k)
      if(a.nodes[k]!=b.nodes[k] || a.neighbors[k] || a.neighbor_edges[k] || a.normal_reference[k])
        return {Status::InvalidInput,expected.expanded_to_primary[i]};
  }
  for(std::size_t i=0;i<p;++i)
    if(sides.primary_to_partner[i]!=expected.primary_to_partner[i] || sides.primary_roles[i]!=expected.primary_roles[i] ||
        !Same(sides.primary_identities[i],expected.primary_identities[i]))return {Status::InvalidInput,i};
  for(std::size_t i=0;i<input.raw_origin_count;++i)
    if(sides.raw_origin_to_primary[i]!=expected.raw_origin_to_primary[i] || !Same(sides.raw_origins[i],expected.raw_origins[i]))
      return {Status::InvalidInput};
  return {Status::Ok};
}
}
Forecast PreflightMixedStarter(const Input& input,const MixedSidesSnapshot& sides,
    const PostGapmTopology& post,Limits limits) noexcept {
  if(input.profile!=Profile::MixedSurface || input.topology!=TopologyPolicy::NativeMixedSurface) {
    Forecast f;f.status=Status::UnsupportedProfile;return f;
  }
  d::Layout layout;
  auto report=d::MakeLayout(input.node_count,input.primary_count,limits,layout,input.topology,
      input.shell_primary_count,input.raw_origin_count);
  if(report.status==Status::Ok)report=Descriptors(input,sides,post);
  if(report.status!=Status::Ok) {Forecast f;f.status=report.status;return f;}
  return layout.forecast;
}
Report BuildStarter(const Input& input,const MixedSidesSnapshot& sides,const PostGapmTopology& post,
    Limits limits,tl::util::HostArena& output,tl::util::HostArena& scratch,Snapshot* published) noexcept {
  if(input.profile!=Profile::MixedSurface || input.topology!=TopologyPolicy::NativeMixedSurface)
    return {Status::UnsupportedProfile};
  d::Layout layout;
  auto report=d::MakeLayout(input.node_count,input.primary_count,limits,layout,input.topology,
      input.shell_primary_count,input.raw_origin_count);
  if(report.status!=Status::Ok)return report;
  if(output.bytes()<layout.forecast.output_bytes || scratch.bytes()<layout.forecast.scratch_bytes)
    return {Status::ResourceLimit};
  report=d::CheckInput(input,layout,output,scratch,published,sizeof(Snapshot));
  if(report.status!=Status::Ok)return report;
  report=Descriptors(input,sides,post);if(report.status!=Status::Ok)return report;
  if(!SourceDisjoint(sides,post,output.data(),output.bytes()) ||
      !SourceDisjoint(sides,post,scratch.data(),scratch.bytes()) ||
      !SourceDisjoint(sides,post,published,sizeof(*published)))return {Status::InvalidInput};
  auto staged=d::Construct(scratch,layout.output);
  auto* points=scratch.Construct<Vector>(layout.points);
  // Reuse the qualified source-input/SH2 validation to check the supplied
  // genuine sides. This comparison never regenerates a post-GAPM partner.
  report=d::Expand(input,staged,points,scratch.Construct<d::Identity>(layout.identities),
      scratch.Construct<d::FaceKey>(layout.face_keys));
  if(report.status!=Status::Ok)return report;
  report=SameSides(input,sides,staged);if(report.status!=Status::Ok)return report;
  std::copy_n(sides.mains,sides.main_count,staged.mains); // Exact validated sides, then primary-only change.
  d::CopyPostGapm(post,staged);
  for(std::size_t i=0;i<input.primary_count;++i) {
    const auto before=staged.mains[i];
    for(unsigned k=0;k<4;++k)staged.mains[i].nodes[k]=before.nodes[post.primary_corners[i].source_corner[k]];
  }
  auto* edges=scratch.Construct<d::Edge>(layout.edges);
  auto* tags=scratch.Construct<int>(layout.tags);
  auto* ids=scratch.Construct<int>(layout.candidate_ids);
  report=d::OrderedNeighbors(input,staged,points,edges,ids,
      scratch.Construct<double>(layout.candidate_angles),scratch.Construct<double>(layout.candidate_sides),
      staged.post_gapm,tags);
  if(report.status!=Status::Ok)return report;
  const auto warnings=report.neighbor_warnings;
  // Ordinary edge buckets are dead. Reuse the same4G extent for the native
  // solid-support-only incidence. tags holds ITAG_S until all reference unions
  // finish; References then reuses it for compact reference labels.
  std::size_t solid_edges=0;
  if(post.final_solid_erosion==SolidErosion::Enabled)
    solid_edges=d::BuildEdges(staged,sides.main_count,edges,staged.post_gapm,d::EdgePopulation::SolidSupport);
  std::size_t references=0,incidence=0;
  report=d::References(input,staged,scratch.Construct<int>(layout.parents),tags,
      scratch.Construct<std::uint32_t>(layout.node_references),references,incidence,
      staged.post_gapm,edges,solid_edges,ids);
  if(report.status!=Status::Ok){report.neighbor_warnings=warnings;return report;}
  // Solid edge queries are now dead; use the checked edge region for REAL4
  // neighbor-normal gathers, as in the legacy source path.
  const auto normals=4*sides.main_count;
  const tl::util::ArenaRegion previous_region{layout.edges.offset,normals,normals*sizeof(StoredNormal)};
  auto* previous=scratch.Construct<StoredNormal>(previous_region);
  report=d::StarterNormals(points,staged,input.primary_count,sides.main_count,references,previous,staged.post_gapm);
  if(report.status!=Status::Ok){report.neighbor_warnings=warnings;return report;}
  auto committed=d::Construct(output,layout.output);
  d::CopyOutput(staged,committed,layout.output);
  Snapshot next;
  next.mains=committed.mains;next.node_count=input.node_count;next.primary_count=input.primary_count;
  next.main_count=sides.main_count;next.expanded_to_primary=committed.expanded_to_primary;
  next.primary_to_partner=committed.primary_to_partner;next.normal_offsets=committed.normal_offsets;
  next.normal_mains=committed.normal_mains;next.normal_incidence_count=incidence;
  next.starter={committed.normals,committed.references,references};next.source_generation=input.source_generation;
  next.profile=input.profile;next.topology=input.topology;next.primary_roles=committed.primary_roles;
  next.primary_role_count=input.primary_count;next.primary_identities=committed.primary_identities;
  next.primary_identity_count=input.primary_count;next.shell_primary_count=input.shell_primary_count;
  next.raw_origins=committed.raw_origins;next.raw_origin_to_primary=committed.raw_origin_to_primary;
  next.raw_origin_count=input.raw_origin_count;next.post_gapm=committed.post_gapm;
  *published=next;
  report={Status::Ok};report.neighbor_warnings=warnings;return report;
}
}
