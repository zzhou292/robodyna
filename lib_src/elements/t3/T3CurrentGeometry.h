// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected C3COOR3/C3EVEC3/C3DERI3, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "T3Geometry.h"
#include "T3KinematicsData.h"
namespace tl::fea::t3::detail {
// Pure element-local work, reused by the later owning force operation. Public
// normalized diagnostics never replace raw source arrays in constitutive work.
struct GeometryWork { Kinematics kinematics; };
TL_T3_HD inline Status CurrentGeometry(const Vec3 (&x)[3],double longest,GeometryWork& output) {
  FrameWork f; const auto status=NativeFrame(x,f); if(status!=Status::kSuccess) return status;
  GeometryWork next; auto& k=next.kinematics; k.frame=f.frame; k.area=f.area;
  const double x2=Project(f.frame,0,f.edge21),y2=Project(f.frame,1,f.edge21),x3=Project(f.frame,0,f.edge31);
  double y3=Project(f.frame,1,f.edge31);
  // Native Y3 floor remains below an explicit scale-aware exclusion band.
  if(!tl::math::Finite(y3)||!(y3>32*Em15+GuardBand*longest)) return Status::kUnsupportedGeometry;
  y3=::copysign(::fmax(Em15,::fabs(y3)),y3);
  k.local_position[1]={x2,y2,0}; k.local_position[2]={x3,y3,0};
  k.derivative[0]=-.5*y3; k.derivative[1]=.5*(x3-x2); k.derivative[2]=-.5*x3;
  k.characteristic_length=CharacteristicLength(x2,x3,y3,k.area); k.area_scale=1;
  if(!Finite(k.local_position[1])||!Finite(k.local_position[2])||!Positive(k.characteristic_length)) return Status::kNonfiniteResult;
  // Both denominators are used by the complete selected C3DEFO3. Reject a
  // nonrepresentable cancellation instead of repairing the native geometry.
  if(k.derivative[0]==0||k.derivative[1]+k.derivative[2]==0) return Status::kUnsupportedGeometry;
  output=next; return Status::kSuccess;
}
} // namespace tl::fea::t3::detail
