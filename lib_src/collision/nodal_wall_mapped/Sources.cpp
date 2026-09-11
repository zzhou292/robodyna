// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "../Q4ParametricContact.h"
#include "../NodalWallWeightStartup.h"
#include "lib_utils/SourceIdentityIndex.h"
#include <algorithm>

namespace tlfea::contact {
namespace m=nodal_wall_mapped;
namespace fe=tl::fea;
using Code=NodalWallDeviceStatus;
namespace {
bool SameWeight(const NodalWallParentWeight& a,const NodalWallParentWeight& b) {
  if(a.parent_element_id!=b.parent_element_id || a.feature_id!=b.feature_id ||
      a.parent_face_id!=b.parent_face_id || a.family!=b.family || a.arity!=b.arity ||
      !m::SameCertificate(a.area,b.area) || !m::SameCertificate(a.share,b.share)) return false;
  for(unsigned l=0;l<4;++l) if(a.nodes[l]!=b.nodes[l]) return false;
  return true;
}
}
NodalWallDeviceReport NodalWallMappedContact::Impl::PrepareSources(const NodalWallWeights& weights,
    const fe::NodalCinWitnessSource& cin,const double* coordinates) {
  const auto& binding=*physical.shells();
  const auto q=binding.qeph_count(),t=binding.t3_count(),b=binding.qbat_count();
  const auto count=q+t+b;
  parents.Resize(count);
  activity.Resize(count);
  snapshots.Resize(rigid.groups().size());
  tl::util::SourceIdentityIndex<0> index;
  index.Prepare(count,[&](std::size_t i) { return weights.parent(i).parent_element_id; });
  const VectorView position{coordinates,weights.global_node_count(),3,1};
  // One bounded reusable native reference; no parent-owned heap or geometry copy.
  Q4ParametricReference quad;
  T3MaterialMeasure triangle;
  for(std::size_t ordinal=0;ordinal<count;++ordinal) {
    const bool is_t3=ordinal>=q && ordinal<q+t;
    const bool is_qbat=ordinal>=q+t;
    const auto family=is_t3?fe::ShellBindingFamily::T3:
        (is_qbat?fe::ShellBindingFamily::Qbat:fe::ShellBindingFamily::Qeph);
    const auto local=is_t3?ordinal-q:(is_qbat?ordinal-q-t:ordinal);
    const auto id=is_t3?binding.t3_source_id(local):
        (is_qbat?binding.qbat_source_id(local):binding.qeph_source_id(local));
    const auto p=index.First(id);
    if(!id || p==SIZE_MAX || parents[p].source_id)
      return {Code::InvalidInput,"Contact does not cover every distinct source shell exactly once",UINT32_MAX,
          static_cast<std::uint32_t>(ordinal)};
    NodalWallParentInput input;
    if(is_t3) {
      SurfaceTriangle parent;
      parent.feature_id=parent.parent_element_id=id;
      parent.interpolation=SurfaceInterpolation::kLinearTriangle;
      for(unsigned l=0;l<3;++l) parent.nodes[l]=physical.mapping()->owner_index(binding.t3_nodes(local)[l]);
      if(PrepareT3MaterialMeasure(position,parent,&triangle)!=SurfaceMeasureStatus::Ok)
        return {Code::GeometryFailure,"Actual T3 reference contact measure rejected",UINT32_MAX,static_cast<std::uint32_t>(p)};
      input.t3=&triangle;
    } else {
      SurfaceQ4 parent;
      parent.feature_id=parent.parent_element_id=id;
      const auto& nodes=is_qbat?binding.qbat_nodes(local):binding.qeph_nodes(local);
      for(unsigned l=0;l<4;++l) parent.nodes[l]=physical.mapping()->owner_index(nodes[l]);
      // Initialize is single-use; reset this fixed-capacity value before the
      // next source parent while reusing the same bounded stack storage.
      quad=Q4ParametricReference{};
      if(quad.Initialize(position,&parent,1).status!=Q4ParametricStatus::Ok)
        return {Code::GeometryFailure,"Actual Q4 reference contact measure rejected",UINT32_MAX,static_cast<std::uint32_t>(p)};
      input.q4=&quad;
    }
    NodalWallParentWeight expected;
    if(nodal_wall_detail::PrepareParentWeight(position.node_count,input,&expected).status!=NodalWallStatus::Ok ||
        !SameWeight(expected,weights.parent(p)))
      return {Code::InvalidInput,"Contact identity, ordered nodes or exact native area differs",UINT32_MAX,static_cast<std::uint32_t>(p)};
    parents[p]={family,static_cast<std::uint32_t>(local),id};
  }
  tl::util::BoundedStartupArray<std::uint32_t,0> roles;
  roles.Resize(weights.global_node_count());
  std::fill_n(roles.data(),roles.size(),UINT32_MAX);
  for(std::size_t g=0;g<rigid.groups().size();++g) {
    const auto& group=rigid.groups()[g];
    for(std::size_t j=0;j<group.member_count;++j)
      roles[rigid.members()[group.member_offset+j].domain_node]=static_cast<std::uint32_t>(g);
  }
  for(std::size_t i=0;i<cin.model->rows().count;++i)
    roles[cin.model->rows().data[i].secondary_domain_node]=UINT32_MAX-1;
  tl::util::BoundedStartupArray<std::size_t,0> surface;
  surface.Resize(weights.node_count());
  for(std::size_t i=0;i<weights.node_count();++i) {
    const auto node=weights.node(i).node;
    surface[i]=node;
    if(roles[node]==UINT32_MAX-1)
      return {Code::InvalidInput,"CIN secondary surface contact requires a separately qualified response",node};
    local.roots[i]=roles[node];
  }
  const auto queried=owner->ValidateFreeRotationalNodes(surface.data(),surface.size());
  if(queried.status!=fe::NodalStatus::Ok)
    return {Code::InvalidInput,"This mapped shell contact profile requires present free world surface DOFs",queried.node};
  return {Code::Ok,"Complete physical shell contact source authenticated"};
}
} // namespace tlfea::contact
