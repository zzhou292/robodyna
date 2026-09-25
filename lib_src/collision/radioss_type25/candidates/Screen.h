// SPDX-License-Identifier: AGPL-3.0-or-later
// Local strict coordinate and diagonal screens in native I25TRIVOX a62b27e6.
#pragma once
#include "PenetrationFilter.h"
namespace tlfea::contact::radioss_type25::candidates {
TL_MATH_HOST_DEVICE inline Status ScreenBounds(const ScreenRow& row,Envelope* output) {
  namespace d=detail;namespace v=tl::math::fixed3;
  if(!output||!d::Nonnegative(row.margin)||!d::Nonnegative(row.curvature)||
     !d::Nonnegative(row.secondary_gap)||!d::Nonnegative(row.main_gap)||
     !tl::math::Finite(row.gap_load)||!d::Nonnegative(row.drad)||
     !d::Nonnegative(row.stored_motion))return Status::InvalidInput;
  for(unsigned i=0;i<4;++i)if(!v::Finite(row.vertices[i]))return Status::InvalidInput;
  const double combined_gap=row.secondary_gap+row.main_gap+row.gap_load;
  const double aaa=row.margin+row.curvature+d::Max(combined_gap,row.drad)+row.stored_motion;
  if(!tl::math::Finite(combined_gap)||!d::Nonnegative(aaa))return Status::NonfiniteResult;
  const auto box=d::Box(row.vertices);
  const Bounds next{v::Subtract(box.minimum,{aaa,aaa,aaa}),v::Add(box.maximum,{aaa,aaa,aaa})};
  if(!v::Finite(next.minimum)||!v::Finite(next.maximum))return Status::NonfiniteResult;
  *output={next,aaa};return Status::Ok;
}
TL_MATH_HOST_DEVICE inline Status EvaluateScreen(const ScreenRow& row,bool* included) {
  namespace d=detail;namespace v=tl::math::fixed3;
  if(!included||!v::Finite(row.secondary))return Status::InvalidInput;
  Envelope envelope;
  const auto status=ScreenBounds(row,&envelope);if(status!=Status::Ok)return status;
  const auto bounds=envelope.bounds;const double aaa=envelope.radius;
  const auto p=row.secondary;
  if(p.x<=bounds.minimum.x||p.x>=bounds.maximum.x||p.y<=bounds.minimum.y||
     p.y>=bounds.maximum.y||p.z<=bounds.minimum.z||p.z>=bounds.maximum.z) {
    *included=false;return Status::Ok;
  }
  const auto diag1=v::Subtract(row.vertices[2],row.vertices[0]);
  const auto diag2=v::Subtract(row.vertices[3],row.vertices[1]);
  const auto s=v::Cross(diag1,diag2),d1=v::Subtract(p,row.vertices[0]),d2=v::Subtract(p,row.vertices[1]);
  const double s2=v::Dot(s,s),dd1=v::Dot(d1,s),dd2=v::Dot(d2,s),product=dd1*dd2;
  if(!v::Finite(diag1)||!v::Finite(diag2)||!v::Finite(s)||!v::Finite(d1)||!v::Finite(d2)||
     !d::Nonnegative(s2)||!tl::math::Finite(dd1)||!tl::math::Finite(dd2)||
     !tl::math::Finite(product))return Status::NonfiniteResult;
  bool result=true;
  if(product>0.) {
    const double first=dd1*dd1,second=dd2*dd2,a2=aaa*aaa*s2;
    if(!d::Nonnegative(first)||!d::Nonnegative(second)||!d::Nonnegative(a2))return Status::NonfiniteResult;
    if(d::Min(first,second)>a2)result=false;
  }
  *included=result;return Status::Ok;
}
} // namespace tlfea::contact::radioss_type25::candidates
