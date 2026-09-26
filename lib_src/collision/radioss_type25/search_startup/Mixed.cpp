// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
#include "../current_normals/Types.h"
#include "../current_normals/MixedSource.h"
#include "../search/Ranges.h"
#include "../../self_contact_filters/Environment.h"
#include <climits>
namespace tlfea::contact::radioss_type25::search_startup::detail {
namespace {
bool Disjoint(const void* a,std::size_t an,const void* b,std::size_t bn) noexcept {
  if(!an||!bn)return true;
  const auto x=reinterpret_cast<std::uintptr_t>(a),y=reinterpret_cast<std::uintptr_t>(b);
  return a&&b&&an<=UINTPTR_MAX-x&&bn<=UINTPTR_MAX-y&&(x+an<=y||y+bn<=x);
}
}
Report AdmitMixed(const Input& in,const Layout&,Limits,const tl::util::HostArena& out,
    const tl::util::HostArena& scratch,const void* result,std::size_t bytes) noexcept {
  namespace r=search::detail;const auto& mesh=in.mesh;const auto& top=in.topology;const auto& p=in.profile;
  if(mesh.profile!=startup::Profile::MixedSurface||mesh.topology!=startup::TopologyPolicy::NativeMixedSurface||
      p.level!=1||p.gap_mode!=1||p.neighbor_removal!=2||p.initial_penetration!=5||p.edge_mode!=0||
      p.thermal_mode!=0||(p.curvature!=0&&p.curvature!=1)||p.partitions!=1||p.gap_load_cards!=LoadCards::Absent||
      (p.initialization!=Initialization::SerialNative&&p.initialization!=Initialization::InvariantNoExpansion))return {Status::UnsupportedProfile};
  if(!self_contact_filters::CompatibleHostArithmetic())return {Status::UnsupportedArithmetic};
  const auto n=mesh.node_count,q=mesh.primary_count,g=in.main_count,s=in.secondary_count;
  std::size_t positions_bytes=0;
  if(!mesh.source_generation||top.source_generation!=mesh.source_generation||top.node_count!=n||top.primary_count!=q||
      mesh.shell_primary_count>q||g!=q+mesh.shell_primary_count||top.main_count!=g||
      top.shell_primary_count!=mesh.shell_primary_count||
      !r::Span(mesh.node_source_ids,n)||!r::Span(mesh.primary,q)||!r::Span(mesh.primary_identities,q)||
      mesh.primary_identity_count!=q||!r::VectorSpan(mesh.positions,n,positions_bytes)||
      !r::Span(mesh.raw_origins,mesh.raw_origin_count)||!r::Span(mesh.raw_origin_to_primary,mesh.raw_origin_count)||
      mesh.raw_origin_count!=top.raw_origin_count||!r::Span(in.secondary,s)||!r::Span(in.main_gaps,g)||!r::Span(static_cast<const std::byte*>(result),bytes)||
      !r::Span(in.auxiliary_rigid_primary_ids,in.auxiliary_rigid_primary_count))return {Status::InvalidInput};
  if(mesh.coordinates!=startup::Coordinates::Native&&mesh.coordinates!=startup::Coordinates::Si)return {Status::InvalidInput};
  units_detail::Factors factors;
  if(mesh.coordinates==startup::Coordinates::Si&&!units_detail::Make(mesh.units,factors))return {Status::InvalidInput};
  current_normals::Topology t;
  t.mains=top.mains;t.nodes=n;t.primary_count=q;t.main_count=g;t.references=top.starter.reference_count;
  t.normal_to_main={top.normal_offsets,t.references+1,top.normal_mains,top.normal_incidence_count};
  t.source_profile=top.profile;t.source_topology=top.topology;t.primary_roles=top.primary_roles;
  t.primary_role_count=top.primary_role_count;t.mixed_maps={top.primary_to_partner,q};
  const auto checked=current_normals::ValidateMixedSource(t,top);
  if(checked.status!=current_normals::Status::Ok)return {checked.status==current_normals::Status::ResourceLimit?
      Status::ResourceLimit:Status::InvalidInput,checked.main,checked.node};
  if(!Disjoint(out.data(),out.bytes(),scratch.data(),scratch.bytes())||
      !Disjoint(out.data(),out.bytes(),result,bytes)||!Disjoint(scratch.data(),scratch.bytes(),result,bytes)||
      !current_normals::detail::MixedSourceDisjoint(top,out.data(),out.bytes())||
      !current_normals::detail::MixedSourceDisjoint(top,scratch.data(),scratch.bytes())||
      !current_normals::detail::MixedSourceDisjoint(top,result,bytes))return {Status::InvalidInput};
  struct Range{const void* data;std::size_t bytes;};
  const Range ranges[]{{&in,sizeof(in)},{&out,sizeof(out)},{&scratch,sizeof(scratch)},
    {mesh.node_source_ids,n*sizeof(std::uint64_t)},{mesh.primary,q*sizeof(startup::PrimaryFace)},
    {mesh.primary_identities,q*sizeof(startup::PrimaryFaceIdentity)},{mesh.positions.data,positions_bytes},
    {mesh.raw_origins,mesh.raw_origin_count*sizeof(startup::PrimaryFaceIdentity)},
    {mesh.raw_origin_to_primary,mesh.raw_origin_count*sizeof(std::uint32_t)},
    {in.secondary,s*sizeof(Secondary)},{in.main_gaps,g*sizeof(double)},
    {in.auxiliary_rigid_primary_ids,in.auxiliary_rigid_primary_count*sizeof(std::uint64_t)}};
  for(const auto& v:ranges)if(!Disjoint(out.data(),out.bytes(),v.data,v.bytes)||
      !Disjoint(scratch.data(),scratch.bytes(),v.data,v.bytes)||!Disjoint(result,bytes,v.data,v.bytes))return {Status::InvalidInput};
  for(std::size_t i=0;i<g;++i)if(!std::isfinite(in.main_gaps[i])||in.main_gaps[i]<0)return {Status::InvalidInput,i};
  constexpr unsigned opposite[]{1,0,3,2};
  for(std::size_t raw=0;raw<mesh.raw_origin_count;++raw) {
    const auto& a=mesh.raw_origins[raw];const auto& b=top.raw_origins[raw];
    if(a.kind!=b.kind||a.physical_parent_id!=b.physical_parent_id||a.local_face!=b.local_face||
        a.origin!=b.origin||a.origin_count!=b.origin_count||mesh.raw_origin_to_primary[raw]!=top.raw_origin_to_primary[raw])
      return {Status::InvalidInput};
  }
  for(std::size_t i=0;i<q;++i) {
    const auto& a=mesh.primary_identities[i];const auto& b=top.primary_identities[i];
    if(a.kind!=b.kind||a.physical_parent_id!=b.physical_parent_id||a.local_face!=b.local_face||
        a.origin!=b.origin||a.origin_count!=b.origin_count||mesh.primary[i].source_id!=top.mains[i].source_id)
      return {Status::InvalidInput,i};
    const auto& face=mesh.primary[i];
    if(face.side_role!=top.primary_roles[i]||
        (face.layout!=ShellLayout::Quad4&&face.layout!=ShellLayout::Triangle3)||
        (face.layout==ShellLayout::Triangle3&&face.nodes[2]!=face.nodes[3]))return {Status::InvalidInput,i};
    std::uint32_t before[4];
    for(unsigned k=0;k<4;++k) {
      if(face.nodes[k]>=n)return {Status::InvalidInput,i};
      before[k]=face.nodes[face.side_role==startup::ShellSideRole::CoatingReversed?opposite[k]:k];
    }
    const auto partner=top.primary_to_partner[i];
    for(unsigned k=0;k<4;++k) {
      if(top.mains[i].nodes[k]!=before[top.post_gapm->primary_corners[i].source_corner[k]])return {Status::InvalidInput,i};
      if(partner&&top.mains[partner-1].nodes[k]!=before[opposite[k]])return {Status::InvalidInput,i};
    }
  }
  return {Status::Ok};
}
} // namespace tlfea::contact::radioss_type25::search_startup::detail
namespace tlfea::contact::radioss_type25::search_startup {
Forecast PreflightComposed(const Input& in,Limits limits) noexcept {
  Forecast failure;
  const bool mixed=in.mesh.profile==startup::Profile::MixedSurface;
  if(mixed&&(in.mesh.topology!=startup::TopologyPolicy::NativeMixedSurface||
      in.mesh.shell_primary_count>in.mesh.primary_count||in.main_count!=in.mesh.primary_count+in.mesh.shell_primary_count)) {failure.status=Status::UnsupportedProfile;return failure;}
  std::size_t native_nodes=0;
  const auto context=in.contributors.tied_interfaces?detail::Context::ComposedBeforeTied:detail::Context::ComposedNoTied;
  const auto check=detail::ResolveContext(in,limits,context,native_nodes);
  if(check.status!=Status::Ok){failure.status=check.status;return failure;}
  detail::Layout layout;const auto result=detail::MakeLayout(in.mesh.node_count,in.mesh.primary_count,
      in.secondary_count,limits,layout,in.auxiliary_rigid_primary_count,mixed?in.main_count:0);
  if(layout.forecast.output_bytes)return layout.forecast;
  failure.status=result.status;return failure;
}
}
