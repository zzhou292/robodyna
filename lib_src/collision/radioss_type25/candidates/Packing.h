// SPDX-License-Identifier: AGPL-3.0-or-later
// Local IGAP1 branch of I25COR3T, OpenRadioss a62b27e6, MYREAL8.
#pragma once
#include "Screen.h"
namespace tlfea::contact::radioss_type25::candidates {
TL_MATH_HOST_DEVICE inline Status PackLocal(const LocalRow& in,PackedRow* output) {
  namespace d=detail;namespace v=tl::math::fixed3;
  const auto& s=in.screen;
  if(!output||!in.secondary_node||in.main_count<=0||!d::Nonnegative(in.previous_dt)||
     !v::Finite(s.secondary)||!v::Finite(in.secondary_velocity))return Status::InvalidInput;
  Bounds unused;double radius;
  const auto status=ScreenBounds(s,&unused,&radius);if(status!=Status::Ok)return status;
  PackedRow next;
  next.symmetry=in.constraint_codes[4];
  if(next.symmetry<0||next.symmetry>7)return Status::InvalidInput;
  for(unsigned i=0;i<4;++i) {
    if(!in.nodes[i]||!v::Finite(in.main_velocities[i])||in.constraint_codes[i]<0||in.constraint_codes[i]>7)
      return Status::InvalidInput;
    for(unsigned j=0;j<i;++j)if(in.nodes[i]==in.nodes[j]&&
       (s.vertices[i].x!=s.vertices[j].x||s.vertices[i].y!=s.vertices[j].y||s.vertices[i].z!=s.vertices[j].z||
        in.main_velocities[i].x!=in.main_velocities[j].x||in.main_velocities[i].y!=in.main_velocities[j].y||
        in.main_velocities[i].z!=in.main_velocities[j].z||in.constraint_codes[i]!=in.constraint_codes[j]))
      return Status::InvalidInput;
    next.nodes[i]=in.nodes[i];next.vertices[i]=s.vertices[i];next.symmetry&=in.constraint_codes[i];
  }
  const auto vb=d::Box(in.main_velocities);
  const auto high=v::Subtract(vb.maximum,in.secondary_velocity),low=v::Subtract(in.secondary_velocity,vb.minimum);
  if(!v::Finite(high)||!v::Finite(low))return Status::NonfiniteResult;
  const double vx=d::Max(high.x,low.x),vy=d::Max(high.y,low.y),vz=d::Max(high.z,low.z);
  const double speed=vx+vy+vz,motion=speed*in.previous_dt;
  const double base=d::Max(s.drad,s.secondary_gap+s.main_gap+s.gap_load);
  next.gap=(1.+1./100.)*(base+s.curvature+motion);
  if(!d::Nonnegative(speed)||!d::Nonnegative(motion)||!d::Nonnegative(next.gap))return Status::NonfiniteResult;
  next.secondary=s.secondary;next.margin=s.margin;next.main_count=in.main_count;next.segment_type=in.segment_type;
  *output=next;return Status::Ok;
}
TL_MATH_HOST_DEVICE inline Status EvaluateLocal(const LocalRow& in,FilterResult* output) {
  if(!output)return Status::InvalidInput;
  bool admitted=false;const auto screen=EvaluateScreen(in.screen,&admitted);
  if(screen!=Status::Ok)return screen;
  // Packing inputs are validated even for screen rejection; malformed source
  // rows must never silently become an empty but successful inventory.
  PackedRow packed;const auto status=PackLocal(in,&packed);if(status!=Status::Ok)return status;
  if(!admitted){*output={};return Status::Ok;}
  return EvaluatePacked(packed,output);
}
} // namespace tlfea::contact::radioss_type25::candidates
