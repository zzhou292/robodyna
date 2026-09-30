// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../radioss_type25_runtime/MixedRuntimeFixture.h"
namespace activity_operands_test {
// Rebuild through the owning Starter producer with a genuinely disabled final
// erosion declaration; never edit an already-produced source snapshot in place.
inline void DisableErosion(type25_source_test::MixedRuntimeSource& source,
    tl::util::HostArena& output,tl::util::HostArena& scratch) {
  namespace n=tlfea::contact::radioss_type25;namespace s=n::startup;
  auto& p=source.physical;auto& post=source.post;
  post.incoming_solid_erosion=post.final_solid_erosion=s::SolidErosion::Disabled;
  s::Input in;in.profile=s::Profile::MixedSurface;in.topology=s::TopologyPolicy::NativeMixedSurface;
  in.node_source_ids=p.ids.data();in.node_count=p.ids.size();
  in.positions={p.positions.data(),std::uint32_t(p.ids.size()),3,1};
  in.primary=p.primary.data();in.primary_count=p.primary.size();in.source_generation=7;
  in.primary_identities=source.identities.data();in.primary_identity_count=source.identities.size();in.shell_primary_count=2;
  in.raw_origins=source.origins.data();in.raw_origin_to_primary=source.origin_map.data();in.raw_origin_count=source.origins.size();
  const auto forecast=s::PreflightMixedStarter(in,source.sides,post);
  type25_source_test::MixedRuntimeSource::Need(forecast.status==s::Status::Ok&&
      output.Initialize(forecast.output_bytes)&&scratch.Initialize(forecast.scratch_bytes),"Disabled erosion Starter forecast");
  s::Snapshot next;
  type25_source_test::MixedRuntimeSource::Need(s::BuildStarter(in,source.sides,post,{},output,scratch,&next).status==s::Status::Ok,
      "Disabled erosion Starter production");
  source.starter=next;p.starter=next;
  for(std::size_t i=0;i<p.mains.size();++i)for(unsigned k=0;k<4;++k){
    p.mains[i].nodes[k]=next.mains[i].nodes[k];p.mains[i].neighbors[k]=next.mains[i].neighbors[k];
    p.mains[i].normal_reference[k]=next.mains[i].normal_reference[k];
    p.mains[i].normal_slot[k]=next.starter.face_normals[4*i+k];}
  p.references.resize(next.starter.reference_count);
  for(std::size_t i=0;i<p.references.size();++i)p.references[i]=next.starter.references[i];
}
} // namespace activity_operands_test
