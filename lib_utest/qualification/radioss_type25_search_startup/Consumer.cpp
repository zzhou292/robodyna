// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/collision/RadiossType25SearchStartup.h"
#include "lib_src/collision/RadiossType25FixedMainStartup.h"
int main() {
  namespace n=tlfea::contact::radioss_type25;
  namespace st=n::startup;namespace s=n::search_startup;
  const std::uint64_t ids[]{1,2,3,4};const double x[]{0,0,0,1,0,0,1,1,0,0,1,0};
  const st::PrimaryFace face{9,n::ShellLayout::Quad4,{0,1,2,3}};
  st::Input mesh;mesh.profile=st::Profile::OrdinaryExteriorFixedMain;mesh.node_source_ids=ids;
  mesh.node_count=4;mesh.positions={x,4,3,1};mesh.primary=&face;mesh.primary_count=1;mesh.source_generation=1;
  const auto first=st::Preflight(4,1);tl::util::HostArena top,temporary;
  if(!top.Initialize(first.output_bytes)||!temporary.Initialize(first.scratch_bytes))return 1;
  st::Snapshot topology;if(st::BuildStarter(mesh,{},top,temporary,&topology).status!=st::Status::Ok)return 2;
  const s::Secondary secondary[]{{0,1,.01},{1,1,.01},{2,1,.01},{3,1,.01}};const double gap[]{.01,.01};
  s::Input in;in.mesh=mesh;in.topology=topology;in.secondary=secondary;in.secondary_count=4;in.main_gaps=gap;in.main_count=2;
  in.contributors.census=s::Census::CompleteDeclaredModel;in.contributors.physical_nodes=4;in.contributors.physical_shells=1;
  auto& p=in.profile;p.level=1;p.gap_mode=1;p.neighbor_removal=2;p.initial_penetration=5;p.edge_mode=0;p.thermal_mode=0;
  p.curvature=0;p.partitions=1;p.initialization=s::Initialization::InvariantNoExpansion;p.gap_load_cards=s::LoadCards::Absent;
  const auto plan=s::Preflight(4,1,4);tl::util::HostArena output,scratch;
  if(!output.Initialize(plan.output_bytes)||!scratch.Initialize(plan.scratch_bytes))return 3;
  s::Snapshot result;if(s::Build(in,{},output,scratch,&result).status!=s::Status::Ok)return 4;
  if(result.removal_count!=0||result.margin<=0||result.source_generation!=1)return 5;
  for(unsigned i=0;i<4;++i)if(result.initial_contact[i]!=0)return 6;
  const std::uint64_t primary_id=5;
  in.contributors.rigid_bodies=1;in.contributors.native_auxiliary_nodes=1;
  in.auxiliary_rigid_primary_ids=&primary_id;in.auxiliary_rigid_primary_count=1;
  const auto more=s::Preflight(in);tl::util::HostArena rigid_output,rigid_scratch;
  if(more.status!=s::Status::Ok||!rigid_output.Initialize(more.output_bytes)||!rigid_scratch.Initialize(more.scratch_bytes))return 7;
  if(s::BuildRigidOnly(in,{},rigid_output,rigid_scratch,&result).status!=s::Status::Ok||result.native_model_nodes!=5)return 8;
  in.contributors.rigid_bodies=0;in.contributors.native_auxiliary_nodes=0;
  in.auxiliary_rigid_primary_ids=nullptr;in.auxiliary_rigid_primary_count=0;
  in.contributors.tied_interfaces=1;in.contributors.cin_links=1;
  s::GeometricSnapshot pending;
  if(s::BuildGeometricBeforeTied(in,{},rigid_output,rigid_scratch,&pending).status!=s::Status::Ok||
      pending.geometry.native_model_nodes!=4||pending.contributors.cin_links!=1)return 9;
  return 0;
}
