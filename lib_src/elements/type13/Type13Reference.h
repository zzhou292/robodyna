// SPDX-License-Identifier: AGPL-3.0-or-later
// OpenRadioss-derived R4BUF3. See qualification/type13/native/source-manifest.json.
#pragma once
#include "Type13Units.h"

namespace tl::fea::type13 {
TL_TYPE13_HD inline Status InitializeReference(WorkingUnits units,const ReferenceInput& input,Reference& output) {
  using namespace tl::math::fixed3;
  detail::UnitFactors factors;
  if(!detail::ResolveUnits(units,factors)||!detail::Nonnegative(input.coordinate_noise)||
     !Unit(input.skew_x)||!Unit(input.skew_y)||::fabs(Dot(input.skew_x,input.skew_y))>1e-12)return Status::InvalidInput;
  for(unsigned i=0;i<3;++i)if(!Finite(input.position[i]))return Status::InvalidInput;
  for(unsigned i=0;i<4;++i)if(input.endpoint_release[i])return Status::UnsupportedScope;
  Reference next;
  const Vec3 chord=Subtract(input.position[1],input.position[0]);
  next.length_native=Norm(chord);
  const double noise=2*::sqrt(3.0)*input.coordinate_noise;
  if(!Finite(next.length_native)||!Finite(noise))return Status::NonfiniteResult;
  if(next.length_native<1e-15||next.length_native<=noise)return Status::DegenerateGeometry;
  const Vec3 seed=Subtract(input.position[2],input.position[0]);
  const double seed2=Dot(seed,seed);
  if(!detail::Positive(seed2))return Status::DegenerateGeometry;
  Vec3 cross=Cross(chord,seed);
  // The source N3 test divides by NRLOC (squared seed length), unlike its
  // fallback test. Do not normalize the seed or replace this with sin(theta).
  next.third_node_alignment=::sqrt(Dot(cross,cross))/seed2/next.length_native;
  if(!Finite(next.third_node_alignment))return Status::NonfiniteResult;
  if(next.third_node_alignment<1e-5) {
    next.branch=FrameBranch::SkewY;
    cross=Cross(chord,input.skew_y);
    const double alignment=::sqrt(Dot(cross,cross)/Dot(input.skew_y,input.skew_y))/next.length_native;
    if(!Finite(alignment))return Status::NonfiniteResult;
    if(alignment<1e-5) { next.branch=FrameBranch::SkewX;cross=Cross(chord,input.skew_x); }
  }
  const Vec3 transverse=Cross(cross,chord);
  const double norm=Norm(transverse);
  if(!detail::Positive(norm))return Status::DegenerateGeometry;
  const Vec3 y=Divide(transverse,norm),x=Divide(chord,next.length_native);
  next.axes=Columns(x,y,Cross(x,y));
  next.length_m=next.length_native*units.length_to_m;
  for(unsigned i=0;i<2;++i)next.position_m[i]=Scale(input.position[i],units.length_to_m);
  if(!Orthonormal(next.axes)||!detail::Positive(next.length_m)||
     !Finite(next.position_m[0])||!Finite(next.position_m[1]))return Status::NonfiniteResult;
  output=next;return Status::Success;
}
} // namespace tl::fea::type13
