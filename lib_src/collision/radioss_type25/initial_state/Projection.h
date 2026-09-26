// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
#include "../selection/BoundaryValues.h"
namespace tlfea::contact::radioss_type25::initial_state::detail {
namespace s=selection::detail;
namespace g=geometry_detail;
namespace v=tl::math::fixed3;
struct PairWork {
  s::Work value;
  selection::NativeRetainedResult projection;
  double gap[4]{};
  unsigned selected=0, boundary_visits=0;
  Vector projected{},boundary_normal{};
  double edge_distance=0;
};
// Source-shared operations: original COR3 frame preparation and PEN3 raw/clamped
// projections have the exact same expression order as these qualified leaves.
// Starter gap/penetration and nearest-sector decisions remain distinct.
TL_MATH_HOST_DEVICE inline bool Project(const PairInput& packet,PairWork& w) {
  const auto& in=packet.geometry;
  s::Prepare(in,w.value);s::RawProjection(w.value,w.projection);
  const unsigned count=w.value.frame.triangle?1:4;
  for(unsigned i=0;i<count;++i) {
    const double la=s::ProjectedSectorPoint(in,w.value,w.projection,i);
    auto& sector=w.projection.sector[i];const unsigned j=(i+1)%4;
    w.gap[i]=g::Min(in.secondary_gap+la*w.value.center_gap+
        sector.clamped_lb*in.main_gap[i]+sector.clamped_lc*in.main_gap[j],in.main_gap_max);
    const double bb=v::Dot(w.value.from_secondary,w.value.normal[i]);w.value.plane_distance[i]=bb;
    if(in.segment_type==0)sector.penetration=bb>0?w.gap[i]+bb:0.;
    else if(in.segment_type>packet.expanded_main_count)
      sector.penetration=bb>0?w.gap[i]+bb:g::Max(0.,w.gap[i]-::sqrt(sector.distance_squared));
    else sector.penetration=bb>0?0.:g::Max(0.,w.gap[i]-::sqrt(sector.distance_squared));
  }
  double nearest=native_constant::ep20,lateral=native_constant::ep20;
  unsigned selected=4;
  if(w.value.frame.triangle) {
    if(w.projection.sector[0].distance_squared<=nearest)selected=0;
  } else {
    const auto& a=w.projection.sector;
    const double dmin=g::Min(g::Min(g::Min(a[0].distance_squared,a[1].distance_squared),a[2].distance_squared),a[3].distance_squared);
    for(unsigned i=0;i<4;++i)if(a[i].distance_squared<=s::onep03*dmin) {
      const auto& q=a[i];const double bb=w.value.plane_distance[i];
      const double distance=q.raw_lb>=0&&q.raw_lc>=0?0.:g::Max(0.,q.distance_squared-bb*bb);
      if(distance<lateral){selected=i;lateral=distance;}
    }
  }
  // Original code would index SUBTRIA0 if its finite sentinel premises fail.
  // This API rejects that undefined input without manufacturing a sector.
  if(selected==4)return false;
  w.selected=selected;
  for(unsigned i=0;i<4;++i)if(i!=selected)w.projection.sector[i].penetration=0;
  auto& q=w.projection.sector[selected];
  if(q.raw_lb<-g::em03||q.raw_lc<-g::em03||q.raw_lb+q.raw_lc>1.+g::em03)q.far=1;
  w.projected=v::Add(in.secondary,v::Scale(w.value.normal[selected],w.value.plane_distance[selected]));
  return true;
}
TL_MATH_HOST_DEVICE inline void FreeCone(const PairInput& packet,PairWork& w) {
  const auto& in=packet.geometry;const auto& f=w.value.frame;const unsigned i=w.selected,j=(i+1)%4;
  auto& q=w.projection.sector[i];
  if(!f.triangle) {
    if(in.neighbors[i]==0) {
      if(v::Dot(v::Subtract(w.projected,f.point[i]),f.normal[i])>=in.secondary_gap&&w.value.shell)q.far=2;
    } else if((in.boundary_ids[i]&&!in.boundary_ids[j])||(!in.boundary_ids[i]&&in.boundary_ids[j])) {
      if(s::OutsideVertex(in,w.value,w.projected,in.boundary_ids[i]?i:j)&&w.value.shell)q.far=2;
    }
    if((q.far==1||w.value.plane_distance[i]<=0)&&s::ConeSide(w.value,i,j,i)<-s::zep01)q.far=2;
  } else {
    const double d1=v::Dot(v::Subtract(w.projected,f.point[0]),f.normal[0]);
    const double d2=v::Dot(v::Subtract(w.projected,f.point[1]),f.normal[1]);
    const double d3=v::Dot(v::Subtract(w.projected,f.point[2]),f.normal[3]);
    if(((!in.neighbors[0]&&d1>=in.secondary_gap)||(!in.neighbors[1]&&d2>=in.secondary_gap)||
        (!in.neighbors[3]&&d3>=in.secondary_gap))&&w.value.shell)q.far=2;
    else {
      unsigned count=0,corner=0;for(unsigned k=0;k<3;++k)if(in.boundary_ids[k]){++count;corner=k;}
      if(count==1&&s::OutsideVertex(in,w.value,w.projected,corner)&&w.value.shell)q.far=2;
    }
    if(q.far==1||w.value.plane_distance[i]<=0) {
      if(in.neighbors[0]&&s::ConeSide(w.value,0,1,0)<-s::zep01)q.far=2;
      if(in.neighbors[1]&&s::ConeSide(w.value,1,2,1)<-s::zep01)q.far=2;
      if(in.neighbors[3]&&s::ConeSide(w.value,2,0,3)<-s::zep01)q.far=2;
    }
  }
  if(q.far==2)q.penetration=0;
}
} // namespace tlfea::contact::radioss_type25::initial_state::detail
