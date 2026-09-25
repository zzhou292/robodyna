// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Projection.h"
namespace tlfea::contact::radioss_type25::selection::detail {
TL_MATH_HOST_DEVICE inline bool OutsideVertex(const NativePairInput& in,const Work& w,
    Vector projected,unsigned corner) {
  const auto a=in.vertex_bisector[corner][0],b=in.vertex_bisector[corner][1];
  const auto delta=v::Subtract(projected,w.frame.point[corner]);
  if(a.x!=0||a.y!=0||a.z!=0||b.x!=0||b.y!=0||b.z!=0)
    return v::Dot(delta,g::Promote(a))>=in.secondary_gap&&
           v::Dot(delta,g::Promote(b))>=in.secondary_gap;
  Vector direction;
  if(!w.frame.triangle)direction=w.frame.arm[corner];
  else direction=v::Subtract(v::Scale(w.frame.point[corner],2.),
      v::Add(w.frame.point[(corner+1)%3],w.frame.point[(corner+2)%3]));
  const double inverse=1./g::Max(g::em20,::sqrt(v::Dot(direction,direction)));
  return v::Dot(delta,direction)*inverse>=in.secondary_gap;
}
TL_MATH_HOST_DEVICE inline double ConeSide(const Work& w,unsigned a,unsigned b,unsigned normal) {
  const auto edge=v::Subtract(w.frame.point[b],w.frame.point[a]);
  // Source PX=edge.z*NNY-edge.y*NNZ etc (normal crossed with edge).
  const auto plane=v::Cross(w.frame.normal[normal],edge);
  const double squared=v::Dot(plane,plane),arm_squared=v::Dot(w.relative[a],w.relative[a]);
  return -v::Dot(w.relative[a],plane)*::sqrt(1./g::Max(native_constant::em30,arm_squared*squared));
}
} // namespace tlfea::contact::radioss_type25::selection::detail
