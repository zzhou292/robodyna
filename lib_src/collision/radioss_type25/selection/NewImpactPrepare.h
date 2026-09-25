// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NewImpactTypes.h"
#include "ImpactIntersection.h"
namespace tlfea::contact::radioss_type25::selection::detail {
TL_MATH_HOST_DEVICE inline int OppositeLocal(const NativeNewImpactInput& in) {
  const int role=in.pair.segment_type;
  return role>0?(role>in.segment_count?role-in.segment_count:role):0;
}
TL_MATH_HOST_DEVICE inline bool SameVectorBits(Vector a,Vector b) {
  return tl::math::SameScalarBits(a.x,b.x)&&tl::math::SameScalarBits(a.y,b.y)&&
      tl::math::SameScalarBits(a.z,b.z);
}
TL_MATH_HOST_DEVICE inline bool SameNormalBits(StoredNormal a,StoredNormal b) {
  return tl::math::SameScalarBits(a.x,b.x)&&tl::math::SameScalarBits(a.y,b.y)&&
      tl::math::SameScalarBits(a.z,b.z);
}
TL_MATH_HOST_DEVICE inline bool Valid(const NativeNewImpactInput& in,
    const NativeGeometryHistory& prior) {
  if(!Valid(in.pair,prior)||in.segment_count<=0||in.pair.local_main>in.segment_count||
     std::int64_t(in.pair.segment_type)<-2*std::int64_t(in.segment_count)||
     std::int64_t(in.pair.segment_type)>2*std::int64_t(in.segment_count)||
     !normal_detail::Nonnegative(in.previous_dt)||!v::Finite(in.secondary_velocity))return false;
  for(unsigned i=0;i<4;++i) {
    if(!v::Finite(in.main_velocity[i]))return false;
    for(unsigned j=0;j<i;++j)if(in.pair.main_node_ids[i]==in.pair.main_node_ids[j]&&
       !SameVectorBits(in.main_velocity[i],in.main_velocity[j]))return false;
  }
  const int opposite=OppositeLocal(in);
  if(in.opposite.local_main!=opposite)return false;
  if(opposite==0)return true; // Native COR22 does not consume partner storage.
  if(opposite==in.pair.local_main||in.opposite.global_main<=0)return false;
  constexpr unsigned permutation[]{1,0,3,2};
  for(unsigned i=0;i<4;++i) {
    // Consistency of supplied IDs only; authentic source binding is the owner
    // prerequisite. These IDs never generate a missing partner descriptor.
    if(in.opposite.main_node_ids[i]!=in.pair.main_node_ids[permutation[i]]||
       !v::Finite(g::Promote(in.opposite.normal_slot[i])))return false;
    for(unsigned k=0;k<2;++k)
      if(!v::Finite(g::Promote(in.opposite.vertex_bisector[i][k])))return false;
    for(unsigned j=0;j<i;++j)if(in.opposite.boundary_ids[i]&&
        in.opposite.boundary_ids[i]==in.opposite.boundary_ids[j])
      for(unsigned k=0;k<2;++k)if(!SameNormalBits(in.opposite.vertex_bisector[i][k],
          in.opposite.vertex_bisector[j][k]))return false;
    for(unsigned j=0;j<4;++j)if(in.opposite.boundary_ids[i]&&
        in.opposite.boundary_ids[i]==in.pair.boundary_ids[j])
      for(unsigned k=0;k<2;++k)if(!SameNormalBits(in.opposite.vertex_bisector[i][k],
          in.pair.vertex_bisector[j][k]))return false;
  }
  return true;
}
struct ImpactWork {
  Work geometry;
  NativePairInput opposite_boundary;
  Vector opposite_normal[5]{};
  double gap[4]{}, volume[4]{};
  int opposite_local = 0;
};
TL_MATH_HOST_DEVICE inline void Prepare(const NativeNewImpactInput& in,ImpactWork& work) {
  Prepare(in.pair,work.geometry);
  work.opposite_local=OppositeLocal(in);
  for(unsigned i=0;i<4;++i) {
    const auto normal=v::Cross(work.geometry.frame.arm[i],work.geometry.frame.arm[(i+1)%4]);
    work.volume[i]=v::Dot(normal,work.geometry.from_secondary);
  }
  if(work.opposite_local==0)return;
  work.opposite_boundary=in.pair;
  constexpr unsigned edge_order[]{0,3,2,1},vertex_order[]{1,0,3,2};
  for(unsigned i=0;i<4;++i) {
    work.opposite_boundary.neighbors[i]=in.opposite.neighbors[edge_order[i]];
    work.opposite_boundary.boundary_ids[i]=in.opposite.boundary_ids[vertex_order[i]];
    for(unsigned j=0;j<2;++j)
      work.opposite_boundary.vertex_bisector[i][j]=in.opposite.vertex_bisector[vertex_order[i]][j];
    // COR22 leaves NB slot3 undefined for T3; none of its selected T3 paths
    // consume that slot. Its private API zero is not a native observation.
    if(!work.geometry.frame.triangle||i!=2)
      work.opposite_normal[i]=g::Promote(in.opposite.normal_slot[edge_order[i]]);
  }
  work.opposite_normal[4]=work.geometry.frame.triangle?work.opposite_normal[3]:
      v::Scale(v::Add(v::Add(v::Add(work.opposite_normal[0],work.opposite_normal[1]),
          work.opposite_normal[2]),work.opposite_normal[3]),.25);
  work.opposite_normal[4]=g::Normalize(work.opposite_normal[4],g::em20);
}
TL_MATH_HOST_DEVICE inline ImpactCrossings Intersections(const NativeNewImpactInput& in,
    const ImpactWork& work) {
  if(work.opposite_local==0&&work.volume[0]<0&&work.volume[1]<0&&
     work.volume[2]<0&&work.volume[3]<0)return {};
  const auto& current=work.geometry.frame.point;
  Vector previous[5],arm[4];
  for(unsigned i=0;i<4;++i)previous[i]=v::Subtract(current[i],v::Scale(in.main_velocity[i],in.previous_dt));
  previous[4]=work.geometry.frame.triangle?previous[2]:
      v::Scale(v::Add(v::Add(v::Add(previous[0],previous[1]),previous[2]),previous[3]),.25);
  const auto old_secondary=v::Subtract(in.pair.secondary,v::Scale(in.secondary_velocity,in.previous_dt));
  const auto old_relative=v::Subtract(previous[4],old_secondary);
  for(unsigned i=0;i<4;++i)arm[i]=v::Subtract(previous[i],previous[4]);
  double old_volume[4];
  for(unsigned i=0;i<4;++i)old_volume[i]=v::Dot(v::Cross(arm[i],arm[(i+1)%4]),old_relative);
  return FindNativeCrossings(current,previous,in.pair.secondary,old_secondary,
      work.volume,old_volume,work.geometry.frame.triangle,work.opposite_local!=0,
      in.pair.initial_contact_flag);
}
} // namespace tlfea::contact::radioss_type25::selection::detail
