// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Stages.h"
#include "../selection/lifecycle/Admission.h"
#include "../startup/RolePolicy.h"
namespace tlfea::contact::radioss_type25::current_normals::detail {
namespace ld=selection::lifecycle::detail;
TL_MATH_HOST_DEVICE inline Report ValidateTopology(const Topology& t,RolePolicy policy=RolePolicy::OrdinaryOnly) {
  if (!t.nodes || t.nodes>UINT32_MAX || !t.primary_count || t.primary_count>INT_MAX/8 ||
      t.main_count!=2*t.primary_count || !t.references || t.references>4*t.main_count)
    return {Status::InvalidInput};
  const bool resolved = policy == RolePolicy::ResolvedShellSides;
  if (resolved) {
    if (t.source_profile!=startup::Profile::ResolvedShellSides ||
        t.source_topology!=startup::TopologyPolicy::NativeResolvedShellSides)
      return {Status::UnsupportedProfile};
    if (t.primary_role_count!=t.primary_count || !ld::Span(t.primary_roles,t.primary_role_count))
      return {Status::InvalidInput};
  } else {
    if (policy!=RolePolicy::OrdinaryOnly || t.primary_roles || t.primary_role_count ||
        (t.source_profile!=startup::Profile::Unspecified &&
         t.source_profile!=startup::Profile::OrdinaryExteriorFixedMain &&
         t.source_profile!=startup::Profile::OrdinaryExteriorMovingMain) ||
        (t.source_topology!=startup::TopologyPolicy::ManifoldTwoSided &&
         t.source_topology!=startup::TopologyPolicy::NativeOrdinaryShell))
      return {Status::UnsupportedProfile};
  }
  if(!ld::Span(t.mains,t.main_count)||!ld::CsrValid(t.normal_to_main,t.references,t.main_count,true))
    return {Status::InvalidInput};
  std::size_t expected=0;
  for(std::size_t i=0;i<t.main_count;++i) {
    const auto& m=t.mains[i];const bool tri=m.nodes[2]==m.nodes[3];const unsigned slots=tri?3:4;
    if(!m.source_id||m.global_id!=int(i+1) ||
        std::int64_t(m.segment_type)<-2*std::int64_t(t.main_count) ||
        std::int64_t(m.segment_type)>2*std::int64_t(t.main_count))
      return {Status::UnsupportedTopology,i};
    for(unsigned k=0;k<4;++k) {
      if(m.nodes[k]>=t.nodes||m.normal_reference[k]<=0||std::size_t(m.normal_reference[k])>t.references||
         m.neighbors[k]<0||std::size_t(m.neighbors[k])>t.main_count||
         (m.neighbors[k]==0?m.neighbor_edges[k]!=0:(m.neighbor_edges[k]<1||m.neighbor_edges[k]>4)))
        return {Status::UnsupportedTopology,i};
      if(tri&&k==3&&m.normal_reference[k]!=m.normal_reference[2])return {Status::UnsupportedTopology,i};
      if(k<slots)for(unsigned j=0;j<k;++j)
        if(m.nodes[k]==m.nodes[j]||m.normal_reference[k]==m.normal_reference[j])return {Status::UnsupportedTopology,i};
    }
    if(tri&&(m.neighbors[2]!=0||m.neighbor_edges[2]!=0))return {Status::UnsupportedTopology,i};
    expected+=slots;
  }
  for(std::size_t i=0;i<t.primary_count;++i) {
    const auto& m=t.mains[i];const auto& opposite=t.mains[t.primary_count+i];
    const auto role = resolved ? t.primary_roles[i] : startup::ShellSideRole::Ordinary;
    if (!startup::role_policy::Valid(role)) return {Status::UnsupportedTopology,i};
    const auto offset = role == startup::ShellSideRole::Ordinary ? std::int64_t{0} : std::int64_t(t.main_count);
    if (std::int64_t(m.segment_type)!=std::int64_t(t.primary_count+i+1)+offset ||
        std::int64_t(opposite.segment_type)!=-(std::int64_t(i+1)+offset) || m.source_id!=opposite.source_id)
      return {Status::UnsupportedTopology,i};
    const bool tri=m.nodes[2]==m.nodes[3];
    constexpr unsigned quad_reverse[]{1,0,3,2},triangle_reverse[]{1,0,2,2};
    for(unsigned k=0;k<4;++k)
      if(opposite.nodes[k]!=m.nodes[tri?triangle_reverse[k]:quad_reverse[k]])return {Status::UnsupportedTopology,i};
    // Opposite sides may have different normal-reference IDs at the SAME node.
    // Do not weld them together; each reference's node identity is checked below.
  }
  const auto& csr=t.normal_to_main;
  if(csr.entry_count!=expected)return {Status::UnsupportedTopology};
  for(std::size_t ref=0;ref<t.references;++ref) {
    std::uint32_t previous=0,node=UINT32_MAX;
    for(auto i=csr.offsets[ref];i<csr.offsets[ref+1];++i) {
      const auto id=csr.entries[i];if(id<=previous)return {Status::UnsupportedTopology,SIZE_MAX,ref};
      previous=id;const auto& m=t.mains[id-1];unsigned found=4;
      const unsigned slots=m.nodes[2]==m.nodes[3]?3:4;
      for(unsigned k=0;k<slots;++k)if(std::size_t(m.normal_reference[k]-1)==ref)found=k;
      if(found==4||(node!=UINT32_MAX&&node!=m.nodes[found]))return {Status::UnsupportedTopology,id-1,ref};
      node=m.nodes[found];
    }
  }
  // Every ordered entry is one legal distinct main/reference incidence. Its
  // exact total equals all unique source corners, proving complete coverage.
  return {Status::Ok};
}
TL_MATH_HOST_DEVICE inline Report Validate(const Input& in,Limits cap,double& length) {
  const auto& t=in.topology;
  if ((in.profile!=Profile::OrdinaryShellLocal && in.profile!=Profile::ResolvedShellSidesLocal) ||
      in.free_roster!=normal_activation::FreeRosterPolicy::FreshComplete)
    return {Status::UnsupportedProfile};
  if(!t.nodes||!t.primary_count||t.primary_count>INT_MAX/8||t.main_count!=2*t.primary_count||
     !t.references||t.references>4*t.main_count||t.nodes>UINT32_MAX||
     in.coefficient_count!=t.main_count||in.active_count!=t.main_count||in.tag_count!=t.nodes||
     in.prior_count!=4*t.main_count||in.free_count>t.main_count)return {Status::InvalidInput};
  if(t.nodes>cap.nodes||t.primary_count>cap.primaries||t.references>cap.references||
     t.normal_to_main.entry_count>cap.incidences)return {Status::ResourceLimit};
  const auto policy = in.profile==Profile::ResolvedShellSidesLocal ? RolePolicy::ResolvedShellSides : RolePolicy::OrdinaryOnly;
  const auto topology=ValidateTopology(t,policy);if(topology.status!=Status::Ok)return topology;
  if(!ld::VectorSpan(in.positions,t.nodes)||!ld::Span(in.main_coefficients,in.coefficient_count)||
     !ld::Span(in.main_active,in.active_count)||!ld::Span(in.node_tag,in.tag_count)||
     !ld::Span(in.prior_normals,in.prior_count)||!ld::Span(in.free_main_ids,in.free_count))return {Status::InvalidInput};
  length=1;
  if(in.coordinates==startup::Coordinates::Si) {
    units_detail::Factors units;if(!units_detail::Make(in.units,units))return {Status::InvalidInput};length=units.length;
  } else if(in.coordinates!=startup::Coordinates::Native)return {Status::UnsupportedProfile};
  for(std::size_t i=0;i<t.nodes;++i)
    if(in.node_tag[i]>1||!tl::math::fixed3::Finite(Position(in,std::uint32_t(i),length)))
      return {Status::InvalidInput,SIZE_MAX,SIZE_MAX,i};
  std::size_t free=0;
  for(std::size_t i=0;i<t.main_count;++i) {
    if(!tl::math::Finite(in.main_coefficients[i])||in.main_active[i]>1)return {Status::InvalidInput,i};
    for(unsigned k=0;k<4;++k)if(!fp::Finite(in.prior_normals[4*i+k]))return {Status::InvalidInput,i};
    if(FreeMain(in,i)) {
      if(free>=in.free_count||in.free_main_ids[free]!=i+1)return {Status::InvalidInput,i};
      ++free;
    }
  }
  return free==in.free_count?Report{Status::Ok}:Report{Status::InvalidInput};
}
} // namespace tlfea::contact::radioss_type25::current_normals::detail
