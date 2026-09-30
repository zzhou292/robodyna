// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Projection.h"
namespace tlfea::contact::radioss_type25::initial_state::detail {
TL_MATH_HOST_DEVICE inline void VertexBoundary(const PairInput& packet,PairWork& w,unsigned corner) {
  const auto& in=packet.geometry;
  const auto a=in.vertex_bisector[corner][0],b=in.vertex_bisector[corner][1];
  const auto delta=v::Subtract(w.projected,w.value.frame.point[corner]);
  const double p1=v::Dot(delta,g::Promote(a)),p2=v::Dot(delta,g::Promote(b));
  // Literal Starter branch, including all-zero bisectors. Engine's fake
  // bisector fallback belongs to a different numerical stage.
  if(p1<in.secondary_gap&&p2<in.secondary_gap)w.boundary_normal=g::Normalize(g::AddStored(a,b),native_constant::em30);
  else if(p1<in.secondary_gap)w.boundary_normal=g::Promote(a);
  else if(p2<in.secondary_gap)w.boundary_normal=g::Promote(b);
  else return;
  w.edge_distance=v::Dot(delta,w.boundary_normal);++w.boundary_visits;
}
TL_MATH_HOST_DEVICE inline void Boundary(const PairInput& packet,PairWork& w) {
  const auto& in=packet.geometry;const auto& f=w.value.frame;const unsigned i=w.selected,j=(i+1)%4;
  if(w.projection.sector[i].penetration==0||!w.value.shell)return;
  if(!f.triangle) {
    if(in.neighbors[i]==0) {
      w.boundary_normal=f.normal[i];w.edge_distance=v::Dot(v::Subtract(w.projected,f.point[i]),w.boundary_normal);
      ++w.boundary_visits;
    } else if((in.boundary_ids[i]&&!in.boundary_ids[j])||(!in.boundary_ids[i]&&in.boundary_ids[j]))
      VertexBoundary(packet,w,in.boundary_ids[i]?i:j);
  } else if(!in.neighbors[0]||!in.neighbors[1]||!in.neighbors[3]) {
    w.edge_distance=in.secondary_gap;
    const unsigned normal[3]{i,j,4},edge[3]{0,1,3},origin[3]{i,j,4};
    for(unsigned k=0;k<3;++k)if(!in.neighbors[edge[k]]) {
      const double distance=v::Dot(v::Subtract(w.projected,f.point[origin[k]]),f.normal[normal[k]]);
      if(distance<w.edge_distance) {
        w.edge_distance=distance;w.boundary_normal=f.normal[normal[k]];
        ++w.boundary_visits; // Every native KBORD append; do not coalesce.
      }
    }
  } else {
    unsigned count=0,corner=0;for(unsigned k=0;k<3;++k)if(in.boundary_ids[k]){++count;corner=k;}
    if(count==1)VertexBoundary(packet,w,corner);
  }
}
TL_MATH_HOST_DEVICE inline void Sharp(const PairInput& packet,PairWork& w) {
  const auto& in=packet.geometry;const auto& f=w.value.frame;const unsigned i=w.selected,j=(i+1)%4;
  auto& q=w.projection.sector[i];const double gap=w.gap[i],bb=w.value.plane_distance[i];
  for(unsigned pass=0;pass<w.boundary_visits;++pass) {
    if(packet.profile.sharp==1) {
      const double main_gap=gap-in.secondary_gap;
      if(w.edge_distance>0&&bb+main_gap<0) {
        const double la=1.-q.clamped_lb-q.clamped_lc;
        const auto point=v::Add(v::Add(v::Scale(f.point[4],la),v::Scale(f.point[i],q.clamped_lb)),v::Scale(f.point[j],q.clamped_lc));
        const auto center=v::Add(point,v::Scale(w.value.normal[i],main_gap));
        w.boundary_normal=v::Subtract(in.secondary,center);
        const double distance=::sqrt(v::Dot(w.boundary_normal,w.boundary_normal));
        if(distance>g::em04)q.penetration=g::Max(0.,in.secondary_gap-distance);
        else {
          w.edge_distance=w.edge_distance-in.secondary_gap;
          q.penetration=-bb<gap+w.edge_distance?g::Max(0.,-w.edge_distance):g::Max(0.,gap+bb);
        }
      } else {
        w.edge_distance=w.edge_distance-in.secondary_gap;
        if(w.edge_distance>=0){q.penetration=0;continue;}
        if(gap+w.edge_distance>0)q.penetration=-bb<gap+w.edge_distance?-w.edge_distance:g::Max(0.,gap+bb);
      }
    } else if(packet.profile.sharp==2) {
      w.edge_distance=w.edge_distance-in.secondary_gap;
      if(w.edge_distance>=0)continue;
      if(gap+w.edge_distance>0) {
        const auto center=v::Subtract(w.projected,v::Scale(w.boundary_normal,gap+w.edge_distance));
        w.boundary_normal=v::Subtract(in.secondary,center);
        const double distance=::sqrt(v::Dot(w.boundary_normal,w.boundary_normal));
        if(distance>g::em04)q.penetration=g::Max(0.,gap-distance);
      }
    }
  }
}
} // namespace tlfea::contact::radioss_type25::initial_state::detail
