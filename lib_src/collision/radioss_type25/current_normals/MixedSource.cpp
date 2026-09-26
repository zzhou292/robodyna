// SPDX-License-Identifier: AGPL-3.0-or-later
#include "MixedSource.h"
#include "Admission.h"
#include "lib_utils/BoundedStartupArray.h"
#include "../../self_contact_filters/Environment.h"
#include <new>
namespace tlfea::contact::radioss_type25::current_normals {
namespace {
namespace s=startup;
namespace ld=selection::lifecycle::detail;
bool Kind(s::PhysicalSupportKind kind) noexcept {
  return kind==s::PhysicalSupportKind::EightSlotSolid || kind==s::PhysicalSupportKind::ShellQuad ||
      kind==s::PhysicalSupportKind::ShellTriangle;
}
bool SameSupport(const s::PostGapmMainSupport& a,const s::PostGapmMainSupport& b) noexcept {
  return a.first.kind==b.first.kind && a.first.source_element_id==b.first.source_element_id &&
      a.second_solid_source_id==b.second_solid_source_id;
}
bool Permutation(const s::Main& main,const s::PrimaryCornerPermutation& value,bool can_reverse) noexcept {
  const bool triangle=main.nodes[2]==main.nodes[3];
  const unsigned identity[]{0,1,2,triangle?2u:3u};
  const unsigned reverse_q[]{3,2,1,0},reverse_t[]{1,0,2,2};
  bool same=true,reverse=true;
  for(unsigned k=0;k<4;++k) {
    same=same && value.source_corner[k]==identity[k];
    reverse=reverse && value.source_corner[k]==(triangle?reverse_t[k]:reverse_q[k]);
  }
  return same || (can_reverse && reverse);
}
Report Validate(const Topology& t,const s::Snapshot& source,std::size_t cap) {
  const auto p=source.primary_count,g=source.main_count,raw=source.raw_origin_count;
  const auto need=MixedSourceValidationBytes(p);
  if(!need || cap>Limits{}.source_validation_bytes || cap<need)return {Status::ResourceLimit};
  if(source.profile!=s::Profile::MixedSurface || source.topology!=s::TopologyPolicy::NativeMixedSurface ||
      t.source_profile!=source.profile || t.source_topology!=source.topology)
    return {Status::UnsupportedProfile};
  if(!source.source_generation || !source.node_count || source.node_count>UINT32_MAX ||
      source.shell_primary_count>p || g!=p+source.shell_primary_count || raw<p || raw>1572864 ||
      source.primary_identity_count!=p || source.primary_role_count!=p || !source.post_gapm ||
      !source.starter.reference_count || source.starter.reference_count>4*g ||
      t.nodes!=source.node_count || t.primary_count!=p || t.main_count!=g ||
      t.references!=source.starter.reference_count || t.mains!=source.mains ||
      t.primary_roles!=source.primary_roles || t.primary_role_count!=p ||
      t.normal_to_main.offsets!=source.normal_offsets || t.normal_to_main.offset_count!=t.references+1 ||
      t.normal_to_main.entries!=source.normal_mains || t.normal_to_main.entry_count!=source.normal_incidence_count ||
      t.mixed_maps.primary_count!=p || !ld::Span(t.mixed_maps.primary_to_partner,p) ||
      !ld::Span(source.primary_to_partner,p) || !ld::Span(source.expanded_to_primary,g) ||
      !ld::Span(source.primary_identities,p) || !ld::Span(source.primary_roles,p) ||
      !ld::Span(source.raw_origins,raw) || !ld::Span(source.raw_origin_to_primary,raw) ||
      !ld::Span(source.post_gapm,std::size_t{1}) ||
      !ld::Span(source.starter.face_normals,4*g) || !ld::Span(source.starter.references,t.references))return {Status::InvalidInput};
  const auto& post=*source.post_gapm;
  if(post.phase!=s::PostGapmPhase::FinalizedBeforeNeighbors || post.source_generation!=source.source_generation ||
      post.primary_count!=p || post.before_shell_count!=p || post.main_count!=g ||
      !ld::Span(post.primary_corners,p) || !ld::Span(post.before_shell,p) || !ld::Span(post.final_support,g))
    return {Status::InvalidInput};
  if((post.incoming_solid_erosion!=s::SolidErosion::Disabled && post.incoming_solid_erosion!=s::SolidErosion::Enabled) ||
      (post.final_solid_erosion!=s::SolidErosion::Disabled && post.final_solid_erosion!=s::SolidErosion::Enabled))
    return {Status::UnsupportedProfile};
  std::size_t expected=0;
  auto report=detail::MainFields(t,false,expected);if(report.status!=Status::Ok)return report;
  tl::util::BoundedStartupArray<std::size_t,0> counts;counts.Resize(p);
  for(std::size_t i=0;i<raw;++i) {
    const auto at=source.raw_origin_to_primary[i];const auto& origin=source.raw_origins[i];
    if(at>=p || origin.origin!=s::PrimaryOrigin::SingleSourceFace || origin.origin_count!=1 ||
        !origin.physical_parent_id || origin.kind!=source.primary_identities[at].kind)
      return {Status::UnsupportedTopology,at};
    if(origin.kind==s::PrimaryFaceKind::Shell) {
      if(origin.local_face)return {Status::UnsupportedTopology,at};
    } else if(origin.kind!=s::PrimaryFaceKind::Solid || !origin.local_face || origin.local_face>6)
      return {Status::UnsupportedTopology,at};
    const auto& primary=source.primary_identities[at];
    if(primary.origin==s::PrimaryOrigin::SingleSourceFace &&
        (primary.physical_parent_id!=origin.physical_parent_id || primary.local_face!=origin.local_face))
      return {Status::UnsupportedTopology,at};
    ++counts[at];
  }
  std::size_t shells=0,internal=0;
  for(std::size_t i=0;i<p;++i) {
    const auto& identity=source.primary_identities[i];const auto& m=source.mains[i];
    const auto& before=post.before_shell[i];const auto& support=post.final_support[i];
    if(!counts[i] || identity.origin_count!=counts[i] || source.expanded_to_primary[i]!=i ||
        m.source_id!=identity.physical_parent_id || !startup::role_policy::Valid(source.primary_roles[i]))
      return {Status::UnsupportedTopology,i};
    if(identity.origin==s::PrimaryOrigin::SingleSourceFace) {
      if(counts[i]!=1 || !m.source_id)return {Status::UnsupportedTopology,i};
    } else if(identity.origin==s::PrimaryOrigin::MultipleOrigins) {
      if(counts[i]<2 || m.source_id || identity.physical_parent_id || identity.local_face)
        return {Status::UnsupportedTopology,i};
    } else return {Status::UnsupportedTopology,i};
    if(before.unique_match_count>2)return {Status::UnsupportedProfile,i};
    if((before.unique_match_count==0 && (before.first_solid_source_id || before.second_solid_source_id)) ||
        (before.unique_match_count && !before.first_solid_source_id) ||
        (before.unique_match_count<2 && before.second_solid_source_id) ||
        (before.second_solid_source_id && before.first_solid_source_id==before.second_solid_source_id))
      return {Status::InvalidInput,i};
    internal+=before.second_solid_source_id!=0;
    if(!Permutation(m,post.primary_corners[i],before.first_solid_source_id && !before.second_solid_source_id))
      return {Status::UnsupportedTopology,i};
    if(!Kind(support.first.kind) || !support.first.source_element_id)return {Status::InvalidInput,i};
    if(support.first.kind==s::PhysicalSupportKind::EightSlotSolid) {
      if(support.first.source_element_id!=before.first_solid_source_id ||
          support.second_solid_source_id!=before.second_solid_source_id)return {Status::InvalidInput,i};
    } else if(support.second_solid_source_id)return {Status::InvalidInput,i};
    const auto partner=source.primary_to_partner[i];
    if(t.mixed_maps.primary_to_partner[i]!=partner)return {Status::InvalidInput,i};
    if(identity.kind==s::PrimaryFaceKind::Solid) {
      if(partner || m.segment_type || source.primary_roles[i]!=s::ShellSideRole::Ordinary)
        return {Status::UnsupportedTopology,i};
      if(identity.origin==s::PrimaryOrigin::SingleSourceFace && (!identity.local_face || identity.local_face>6))
        return {Status::UnsupportedTopology,i};
    } else if(identity.kind==s::PrimaryFaceKind::Shell) {
      ++shells;
      if(identity.local_face || partner<=p || partner>g || source.expanded_to_primary[partner-1]!=i ||
          support.first.kind==s::PhysicalSupportKind::EightSlotSolid)
        return {Status::UnsupportedTopology,i};
      const auto& other=source.mains[partner-1];
      const std::int64_t offset=source.primary_roles[i]==s::ShellSideRole::Ordinary?0:std::int64_t(g);
      if(std::int64_t(m.segment_type)!=std::int64_t(partner)+offset ||
          std::int64_t(other.segment_type)!=-(std::int64_t(i+1)+offset) || other.source_id!=m.source_id ||
          !SameSupport(support,post.final_support[partner-1]))return {Status::UnsupportedTopology,i};
      // Both permitted primary permutations are involutions. Reconstruct its
      // pre-GAPM words before validating the unchanged SH2 partner.
      std::uint32_t previous[4];
      for(unsigned k=0;k<4;++k)previous[k]=m.nodes[post.primary_corners[i].source_corner[k]];
      constexpr unsigned q[]{1,0,3,2},tri[]{1,0,2,2};
      const bool triangle=m.nodes[2]==m.nodes[3];
      for(unsigned k=0;k<4;++k)
        if(other.nodes[k]!=previous[triangle?tri[k]:q[k]])return {Status::UnsupportedTopology,i};
    } else return {Status::UnsupportedTopology,i};
  }
  if(shells!=source.shell_primary_count || internal!=post.pre_shell_internal_count ||
      post.final_solid_erosion!=(internal?post.incoming_solid_erosion:s::SolidErosion::Disabled))
    return {Status::InvalidInput};
  // Every appended main is the unique declared partner of its mapped primary.
  for(std::size_t i=p;i<g;++i) {
    const auto primary=source.expanded_to_primary[i];
    if(primary>=p || source.primary_to_partner[primary]!=i+1)return {Status::UnsupportedTopology,i};
  }
  return detail::ReferenceIncidence(t,expected);
}
}
std::size_t MixedSourceValidationBytes(std::size_t p) noexcept {
  if(!p || p>1048576)return 0;
  return sizeof(tl::util::BoundedStartupArray<std::size_t,0>)+p*sizeof(std::size_t)+64;
}
Report ValidateMixedSource(const Topology& t,const startup::Snapshot& source,std::size_t cap) noexcept {
  try { return Validate(t,source,cap); }
  catch(const std::bad_alloc&) { return {Status::ResourceLimit}; }
}
namespace detail {
bool MixedSourceDisjoint(const startup::Snapshot& s,const void* target,std::size_t bytes) noexcept {
  namespace g=geometry_detail;const auto& p=*s.post_gapm;
  const g::Range reads[]{
    {&s,sizeof(s),alignof(startup::Snapshot)},{&p,sizeof(p),alignof(startup::PostGapmTopology)},
    {s.starter.face_normals,4*s.main_count*sizeof(StoredNormal),alignof(StoredNormal)},
    {s.starter.references,s.starter.reference_count*sizeof(startup::NormalReference),alignof(startup::NormalReference)},
    {s.expanded_to_primary,s.main_count*sizeof(std::uint32_t),alignof(std::uint32_t)},
    {s.primary_to_partner,s.primary_count*sizeof(std::uint32_t),alignof(std::uint32_t)},
    {s.primary_identities,s.primary_count*sizeof(startup::PrimaryFaceIdentity),alignof(startup::PrimaryFaceIdentity)},
    {s.raw_origins,s.raw_origin_count*sizeof(startup::PrimaryFaceIdentity),alignof(startup::PrimaryFaceIdentity)},
    {s.raw_origin_to_primary,s.raw_origin_count*sizeof(std::uint32_t),alignof(std::uint32_t)},
    {p.primary_corners,p.primary_count*sizeof(startup::PrimaryCornerPermutation),alignof(startup::PrimaryCornerPermutation)},
    {p.before_shell,p.before_shell_count*sizeof(startup::PreShellSolidSupport),alignof(startup::PreShellSolidSupport)},
    {p.final_support,p.main_count*sizeof(startup::PostGapmMainSupport),alignof(startup::PostGapmMainSupport)}};
  for(const auto& value:reads)if(!g::Disjoint({target,bytes,1},value))return false;
  return true;
}
Report PlanMixed(const Input& in,const startup::Snapshot& source,Limits limits,Layout& out,double& length) noexcept {
  const auto& t=in.topology;
  if(in.profile!=Profile::MixedSurfaceLocal || in.free_roster!=normal_activation::FreeRosterPolicy::FreshComplete)
    return {Status::UnsupportedProfile};
  if(!self_contact_filters::CompatibleHostArithmetic())return {Status::UnsupportedArithmetic};
  if(!t.nodes || !t.primary_count || t.primary_count>INT_MAX/8 || t.main_count<t.primary_count ||
      t.main_count>2*t.primary_count || !t.references || t.references>4*t.main_count || t.nodes>UINT32_MAX ||
      in.coefficient_count!=t.main_count || in.active_count!=t.main_count || in.tag_count!=t.nodes ||
      in.prior_count!=4*t.main_count || in.free_count>t.main_count)return {Status::InvalidInput};
  if(t.nodes>limits.nodes || t.primary_count>limits.primaries || t.references>limits.references ||
      t.normal_to_main.entry_count>limits.incidences)return {Status::ResourceLimit};
  auto report=ValidateMixedSource(t,source,limits.source_validation_bytes);
  if(report.status!=Status::Ok)return report;
  report=DynamicFields(in,length);if(report.status!=Status::Ok)return report;
  report=MakeLayout(in,limits,out);
  if(report.status==Status::Ok)out.forecast.source_validation_bytes=MixedSourceValidationBytes(t.primary_count);
  return report;
}
}
Report Preflight(const Input& in,const startup::Snapshot& source,Limits limits,Forecast& out) noexcept {
  detail::Layout layout;double length=1;
  const auto report=detail::PlanMixed(in,source,limits,layout,length);
  if(report.status!=Status::Ok)return report;
  if(!geometry_detail::RangeValid({&out,sizeof(out),alignof(Forecast)}) ||
      !detail::InputDisjoint(in,&out,sizeof(out)) || !detail::MixedSourceDisjoint(source,&out,sizeof(out)))
    return {Status::InvalidInput};
  out=layout.forecast;return {Status::Ok};
}
Report Evaluate(const Input& in,const startup::Snapshot& source,Limits limits,void* scratch,
    std::size_t bytes,Output output) noexcept {
  detail::Layout layout;double length=1;
  auto report=detail::PlanMixed(in,source,limits,layout,length);
  if(report.status!=Status::Ok)return report;
  report=detail::Storage(in,layout,scratch,bytes,output);if(report.status!=Status::Ok)return report;
  if(!detail::MixedSourceDisjoint(source,scratch,layout.forecast.scratch_bytes) ||
      !detail::MixedSourceDisjoint(source,output.face_normals,output.normal_count*sizeof(StoredNormal)) ||
      !detail::MixedSourceDisjoint(source,output.references,output.reference_count*sizeof(startup::NormalReference)))
    return {Status::InvalidInput};
  return detail::Execute(in,layout,length,scratch,output);
}
}
