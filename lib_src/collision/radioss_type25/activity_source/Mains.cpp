// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
#include "../runtime/physical_main/Origins.h"
#include "../selection/lifecycle/Admission.h"
namespace tlfea::contact::radioss_type25::activity_source::detail {
namespace {
namespace ld=selection::lifecycle::detail;
bool Valid(Controls c) {
  return (c.deletion==Deletion::Disabled||c.deletion==Deletion::AllSupports||c.deletion==Deletion::AnySupport)&&
      (c.solid_erosion==startup::SolidErosion::Disabled||c.solid_erosion==startup::SolidErosion::Enabled);
}
std::uint32_t Shell(const pm::Index& index,std::uint64_t id) {
  const auto row=index.ShellOrdinal(id);return row<UINT32_MAX?static_cast<std::uint32_t>(row):NoParent;
}
std::uint32_t Solid(const pm::Index& index,std::uint64_t id,std::size_t shells) {
  const auto row=index.SolidOrdinal(id);
  return row!=SIZE_MAX&&shells<UINT32_MAX&&row<UINT32_MAX-shells?
      static_cast<std::uint32_t>(shells+row):NoParent;
}
}
TransactionReport CheckSource(const tl::fea::ShellPhysicalBinding& physical,Source source,
    Controls controls,std::size_t cap) {
  if(!Valid(controls))return Fail(S::UnsupportedProfile,"Unknown source deletion or erosion control");
  if(source.mixed) {
    if(!source.post||source.ordinary||source.post->final_solid_erosion!=controls.solid_erosion)
      return Fail(S::SourceMismatch,"Deletion controls differ from the final post-GAPM source");
    return runtime_detail::ValidateMixedPhysicalMains(physical,*source.mixed,*source.post,cap).report;
  }
  if(!source.ordinary||source.post||controls.solid_erosion!=startup::SolidErosion::Disabled)
    return Fail(S::UnsupportedProfile,"Ordinary shell source cannot claim solid erosion");
  const auto& input=*source.ordinary;const auto& s=input.selection;const auto p=input.primary_main_count;
  if(!p||p>INT_MAX/2||!s.generation||s.node_count!=physical.domain()->node_count()||s.main_count!=2*p||
      !ld::Span(s.mains,2*p)||!ld::Span(input.primary_parent_ids,p))
    return Fail(S::InvalidInput,"Incomplete ordinary physical-main source");
  const auto forecast=pm::Index::Preflight(physical,cap);
  if(forecast.report.status!=S::Ok)return forecast.report;
  tl::util::BoundedArenaLayout budget(cap);tl::util::ArenaRegion ignored;
  using Entry=tl::util::SourceIdentityIndex<0>::Entry;
  if(!budget.Append<std::byte>(forecast.bytes,ignored)||!budget.Append<Entry>(p,ignored)||
      !budget.Append<std::byte>(128,ignored))return Fail(S::ResourceLimit,"Ordinary source lookup exceeds cap");
  pm::Index index;auto status=index.Initialize(physical,cap);if(status.status!=S::Ok)return status;
  tl::util::SourceIdentityIndex<0> selected;
  selected.Prepare(p,[&](auto i){return input.primary_parent_ids[i];});
  for(std::size_t i=0;i<p;++i) {
    if(selected.First(input.primary_parent_ids[i])!=i)
      return Fail(S::SourceMismatch,"Repeated ordinary physical primary",i);
    pm::Face nodes;bool triangle=false;
    if(!index.Shell(input.primary_parent_ids[i],nodes,triangle))
      return Fail(S::SourceMismatch,"Ordinary main has no genuine physical shell",i);
    const auto& main=s.mains[i];const auto& opposite=s.mains[p+i];
    if(main.global_id!=int(i+1)||opposite.global_id!=int(p+i+1)||
       main.segment_type!=int(p+i+1)||opposite.segment_type!=-int(i+1))
      return Fail(S::SourceMismatch,"Ordinary opposite main role differs",i);
    for(unsigned k=0;k<4;++k) {
      const unsigned reverse=k==0?1:k==1?0:triangle?2:k==2?3:2;
      if(main.nodes[k]!=nodes[k]||opposite.nodes[k]!=nodes[reverse])
        return Fail(S::SourceMismatch,"Ordinary ordered source connectivity differs",i);
    }
  }
  return Ok();
}
TransactionReport WriteMains(const pm::Index& index,Source source,std::size_t shells,
    MainSupport* mains,Origin* origins) {
  if(source.ordinary) {
    const auto& input=*source.ordinary;const auto p=input.primary_main_count;
    for(std::size_t i=0;i<p;++i) {
      const auto parent=Shell(index,input.primary_parent_ids[i]);
      if(parent==NoParent)return Fail(S::SourceMismatch,"Ordinary main source identity disappeared",i);
      mains[i]={parent,NoParent};mains[p+i]=mains[i];
      origins[i]={parent,static_cast<std::uint32_t>(i),0};
    }return Ok();
  }
  const auto& mixed=*source.mixed;const auto& post=*source.post;
  for(std::size_t i=0;i<mixed.main_count;++i) {
    const auto& support=post.final_support[i];
    const bool solid=support.first.kind==startup::PhysicalSupportKind::EightSlotSolid;
    auto& out=mains[i];out.first=solid?Solid(index,support.first.source_element_id,shells):
        Shell(index,support.first.source_element_id);
    out.second=support.second_solid_source_id?Solid(index,support.second_solid_source_id,shells):NoParent;
    if(out.first==NoParent||(support.second_solid_source_id&&out.second==NoParent))
      return Fail(S::SourceMismatch,"Final main support identity disappeared",i);
  }
  for(std::size_t i=0;i<mixed.raw_origin_count;++i) {
    const auto& raw=mixed.raw_origins[i];
    const auto parent=raw.kind==startup::PrimaryFaceKind::Solid?Solid(index,raw.physical_parent_id,shells):Shell(index,raw.physical_parent_id);
    if(parent==NoParent)return Fail(S::SourceMismatch,"Raw main origin identity disappeared",i);
    origins[i]={parent,mixed.raw_origin_to_primary[i],raw.local_face};
  }
  return Ok();
}
startup::MixedSidesSnapshot Mixed(const startup::Snapshot& s) {
  startup::MixedSidesSnapshot out;
  out.mains=s.mains;out.node_count=s.node_count;out.primary_count=s.primary_count;out.main_count=s.main_count;
  out.shell_primary_count=s.shell_primary_count;out.expanded_to_primary=s.expanded_to_primary;
  out.primary_to_partner=s.primary_to_partner;out.primary_roles=s.primary_roles;out.primary_identities=s.primary_identities;
  out.raw_origins=s.raw_origins;out.raw_origin_to_primary=s.raw_origin_to_primary;
  out.raw_origin_count=s.raw_origin_count;out.source_generation=s.source_generation;return out;
}
}
