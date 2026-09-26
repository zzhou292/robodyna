// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../PhysicalMainSource.h"
#include "Origins.h"
#include "../../selection/lifecycle/Admission.h"
#include "lib_utils/BoundedStartupArray.h"
#include <new>
namespace tlfea::contact::radioss_type25::runtime_detail {
namespace {
namespace pm=physical_main;namespace ld=lifecycle::detail;
using S=TransactionStatus;
PhysicalMainValidation Fail(S status,const char* text,std::size_t row=SIZE_MAX) {
  return {{status,text,row}};
}
pm::Face Nodes(const startup::Main& main) {
  return {main.nodes[0],main.nodes[1],main.nodes[2],main.nodes[3]};
}
template<class View>
PhysicalMainValidation Validate(const tl::fea::ShellPhysicalBinding& physical,const View& source,
    const startup::PostGapmTopology& post,std::size_t cap) {
  const auto p=source.primary_count,g=source.main_count,raw=source.raw_origin_count;
  const auto forecast=pm::Index::Preflight(physical,cap);
  if(forecast.report.status!=S::Ok)return {forecast.report};
  if(!p||p>1048576||source.shell_primary_count>p||g!=p+source.shell_primary_count||
      !raw||raw>1572864||!source.source_generation||source.node_count!=physical.domain()->node_count()||
      post.phase!=startup::PostGapmPhase::FinalizedBeforeNeighbors||post.primary_count!=p||
      post.before_shell_count!=p||post.main_count!=g||post.source_generation!=source.source_generation||
      !ld::Span(source.mains,g)||!ld::Span(source.expanded_to_primary,g)||
      !ld::Span(source.primary_identities,p)||!ld::Span(source.raw_origins,raw)||
      !ld::Span(source.raw_origin_to_primary,raw)||!ld::Span(post.before_shell,p)||
      !ld::Span(post.final_support,g)||!ld::Span(post.primary_corners,p))
    return Fail(S::InvalidInput,"Incomplete mixed physical-main source descriptors");
  tl::util::BoundedArenaLayout budget(cap);tl::util::ArenaRegion ignored;
  if(!budget.Append<std::byte>(forecast.bytes,ignored)||
      !budget.Append<std::size_t>(p,ignored)||!budget.Append<std::byte>(64,ignored)||
      !budget.Append<std::byte>(sizeof(tl::util::BoundedStartupArray<std::size_t,0>),ignored))
    return Fail(S::ResourceLimit,"Complete origin count and physical index exceed host cap");
  pm::Index index;auto report=index.Initialize(physical,cap);
  if(report.status!=S::Ok)return {report};
  tl::util::BoundedStartupArray<std::size_t,0> origins;origins.Resize(p);
  for(std::size_t main=0;main<g;++main) {
    const auto& face=source.mains[main];const auto parent=source.expanded_to_primary[main];
    if(face.global_id!=int(main+1)||parent>=p||(main<p&&parent!=main))
      return Fail(S::SourceMismatch,"Mixed contact-main identity or source map differs",main);
    if(face.source_id!=source.primary_identities[parent].physical_parent_id)
      return Fail(S::SourceMismatch,"Expanded main source identity differs from its primary",main);
    const auto nodes=Nodes(face);const unsigned count=nodes[2]==nodes[3]?3:4;
    for(unsigned slot=0;slot<count;++slot) {
      if(nodes[slot]>=source.node_count)return Fail(S::SourceMismatch,"Mixed main node is outside the physical domain",main);
      for(unsigned before=0;before<slot;++before)
        if(nodes[before]==nodes[slot])return Fail(S::SourceMismatch,"Degenerate mixed physical face",main);
    }
    if(!pm::SameFace(nodes,Nodes(source.mains[parent])))
      return Fail(S::SourceMismatch,"Expanded main is not the same physical source face",main);
    if(!pm::Support(index,post.final_support[main],nodes))
      return Fail(S::SourceMismatch,"Final IELEM owner is not a genuine incident physical parent",main);
  }
  for(std::size_t row=0;row<raw;++row) {
    const auto parent=source.raw_origin_to_primary[row];
    if(parent>=p)return Fail(S::SourceMismatch,"Raw origin map is outside the mixed primary prefix",row);
    const auto& origin=source.raw_origins[row];
    if(origin.kind!=source.primary_identities[parent].kind||
        !pm::Origin(index,origin,Nodes(source.mains[parent])))
      return Fail(S::SourceMismatch,"Raw contact origin is not its declared physical face",row);
    ++origins[parent];
  }
  for(std::size_t parent=0;parent<p;++parent) {
    const auto& identity=source.primary_identities[parent];const auto& before=post.before_shell[parent];
    if(!origins[parent]||identity.origin_count!=origins[parent])
      return Fail(S::SourceMismatch,"Mixed source does not retain every origin",parent);
    if(identity.origin==startup::PrimaryOrigin::MultipleOrigins) {
      if(origins[parent]<2||identity.physical_parent_id||identity.local_face||source.mains[parent].source_id)
        return Fail(S::SourceMismatch,"Multiple-origin face has a fabricated representative identity",parent);
    } else if(identity.origin!=startup::PrimaryOrigin::SingleSourceFace||origins[parent]!=1||
        source.mains[parent].source_id!=identity.physical_parent_id||
        !pm::Origin(index,identity,Nodes(source.mains[parent])))
      return Fail(S::SourceMismatch,"Single-origin primary identity differs from its real source",parent);
    const auto face=Nodes(source.mains[parent]);
    if((before.first_solid_source_id&&!pm::SolidSupport(index,before.first_solid_source_id,face))||
        (before.second_solid_source_id&&(!before.first_solid_source_id||
          before.first_solid_source_id==before.second_solid_source_id||
          !pm::SolidSupport(index,before.second_solid_source_id,face))))
      return Fail(S::SourceMismatch,"Pre-shell INSOL support differs from physical incidence",parent);
  }
  return {{S::Ok,"Every raw face and declared support belongs to the actual physical source"},budget.bytes()};
}
template<class View>
PhysicalMainValidation Checked(const tl::fea::ShellPhysicalBinding& physical,const View& view,
    const startup::PostGapmTopology& post,std::size_t cap) noexcept try {
  return Validate(physical,view,post,cap);
} catch(const std::bad_alloc&) {return Fail(S::ResourceLimit,"Mixed physical source index allocation failed");}
}
PhysicalMainValidation ValidateMixedPhysicalMains(const tl::fea::ShellPhysicalBinding& physical,
    const startup::MixedSidesSnapshot& source,const startup::PostGapmTopology& post,std::size_t cap) noexcept {
  return Checked(physical,source,post,cap);
}
PhysicalMainValidation ValidateMixedPhysicalMains(const tl::fea::ShellPhysicalBinding& physical,
    const startup::Snapshot& source,std::size_t cap) noexcept {
  if(source.profile!=startup::Profile::MixedSurface||source.topology!=startup::TopologyPolicy::NativeMixedSurface||
      !source.post_gapm||source.primary_identity_count!=source.primary_count)
    return Fail(S::UnsupportedProfile,"A genuine post-GAPM mixed Starter source is required");
  return Checked(physical,source,*source.post_gapm,cap);
}
} // namespace tlfea::contact::radioss_type25::runtime_detail
