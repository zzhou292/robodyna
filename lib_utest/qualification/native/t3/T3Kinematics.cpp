#include "T3Kinematics.h"
#include "T3EngineContext.h"
#include "T3Geometry.h"
#include <algorithm>
#include <cmath>

namespace tl::qualification::t3 {
namespace detail {
std::mutex& NativeEngineContext() { static std::mutex context; return context; }
}  // namespace detail
namespace {
bool Finite(Vec3 v) { return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z); }
void Pack(const std::array<Vec3,3>& v,std::array<double,9>& out) {
  for (unsigned n=0;n<3;++n) { out[3*n]=v[n].x; out[3*n+1]=v[n].y; out[3*n+2]=v[n].z; }
}
}  // namespace

Status EvaluatePrescribed(const Reference& reference,const PrescribedInterval& input,Kinematics& output) noexcept {
  if (!reference.prepared() || !input.sample_index || !std::isfinite(input.base_time) || input.base_time<0 ||
      !std::isfinite(input.dt) || input.dt<=0 || .25*input.dt<=0) return Status::kInvalidInput;
  const double endpoint=input.base_time+input.dt,midpoint=input.base_time+.5*input.dt;
  if (!std::isfinite(endpoint) || !(input.base_time<midpoint && midpoint<endpoint)) return Status::kInvalidInput;
  for (unsigned n=0;n<3;++n) {
    if (!Finite(input.position[n]) || !Finite(input.velocity[n]) || !Finite(input.angular_velocity[n]))
      return Status::kInvalidInput;
    for (double v:{input.position[n].x,input.position[n].y,input.position[n].z})
      if (std::abs(v)>kMaximumCoordinate) return Status::kInvalidInput;
  }
  if (!detail::SupportedGeometry(input.position)) return Status::kUnsupportedGeometry;
  std::array<double,9> x{},v{},omega{};
  Pack(input.position,x); Pack(input.velocity,v); Pack(input.angular_velocity,omega);
  std::array<double,detail::kKinematicsValues> values{};
  int status=-1;
  try {
    const std::lock_guard<std::mutex> lock(detail::NativeEngineContext());
    detail::t3_r2_kinematics(x.data(),v.data(),omega.data(),&input.dt,values.data(),&status);
  } catch (...) { return Status::kNativeFailure; }
  if (status==1) return Status::kUnsupportedGeometry;
  if (status!=0) return Status::kNativeFailure;
  if (!std::all_of(values.begin(),values.end(),[](double a){return std::isfinite(a);}))
    return Status::kNonfiniteResult;
  Kinematics next;
  std::copy_n(values.begin(),9,next.frame.v);
  if (!detail::ProperFrame(next.frame)) return Status::kNativeFailure;
  next.area=values[9]; next.characteristic_length=values[10]; next.area_scale=values[11];
  if (next.area<=0 || next.characteristic_length<=0 || next.area_scale!=1) return Status::kNativeFailure;
  next.local_position={{{0,0,0},{values[12],values[13],0},{values[14],values[15],0}}};
  std::copy_n(values.begin()+16,3,next.derivative.begin());
  std::copy_n(values.begin()+19,8,next.raw_rate.begin());
  std::copy_n(values.begin()+27,3,next.corrected_velocity_difference.begin());
  std::copy_n(values.begin()+30,8,next.normalized_rate.begin());
  next.base_time=input.base_time; next.position_time=endpoint; next.velocity_time=midpoint;
  next.dt=input.dt; next.sample_index=input.sample_index; next.valid=true;
  output=next; return Status::kSuccess;
}
}  // namespace tl::qualification::t3
