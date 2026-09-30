#include "T3ForceReference.h"
#include "T3EngineContext.h"
#include <algorithm>
#include <cmath>

namespace tl::qualification::t3 {
Status EvaluateForce(const Reference& r,const History& base,
                     const PrescribedInterval& in,ForceTrial& output) noexcept {
  if(!base.matches_reference(r)||!detail::ValidHistoryValues(base.data())||
     !std::isfinite(base.stamp().time)||base.stamp().time<0||
     in.base_time!=base.stamp().time||base.stamp().sample_index==UINT64_MAX||
     in.sample_index!=base.stamp().sample_index+1) return Status::kInvalidInput;
  const auto checked=detail::CheckPrescribedInput(r,in);
  if(checked!=Status::kSuccess) return checked;
  std::array<double,9> x{},v{},omega{};
  detail::PackVectors(in.position,x); detail::PackVectors(in.velocity,v);
  detail::PackVectors(in.angular_velocity,omega);
  const auto& p=r.data().input;
  const std::array<double,4> material{p.density,p.young_modulus,p.poisson_ratio,p.thickness};
  std::array<double,detail::kHistoryValues> history{}; detail::PackHistory(base.data(),history);
  std::array<double,detail::kForceValues> values{};
  int status=-1;
  try {
    const std::lock_guard<std::mutex> lock(detail::NativeEngineContext());
    detail::t3_r3_force(x.data(),v.data(),omega.data(),material.data(),history.data(),
                       &in.dt,values.data(),&status);
  } catch(...) { return Status::kNativeFailure; }
  if(status==1) return Status::kUnsupportedGeometry;
  if(status==2) return Status::kNonfiniteResult;
  if(status!=0) return Status::kNativeFailure;
  if(!std::all_of(values.begin(),values.end(),[](double a){return std::isfinite(a);}))
    return Status::kNonfiniteResult;
  ForceTrial next;
  std::array<double,detail::kKinematicsValues> geometry{};
  std::copy_n(values.begin(),geometry.size(),geometry.begin());
  const auto decoded=detail::UnpackKinematics(geometry,in,next.kinematics);
  if(decoded!=Status::kSuccess) return decoded;
  const auto h=detail::UnpackHistory(values.data()+38);
  if(PreparePrescribedHistory(r,h,{in.base_time+in.dt,in.sample_index},next.proposed_history)!=Status::kSuccess)
    return Status::kNonfiniteResult;
  for(unsigned n=0;n<3;++n) {
    const auto f=64+3*n,c=73+3*n;
    next.internal_force[n]={values[f],values[f+1],values[f+2]};
    next.internal_couple[n]={values[c],values[c+1],values[c+2]};
  }
  auto& d=next.diagnostics;
  d.effective_thickness=values[82]; d.native_sound_speed=values[83];
  d.membrane_viscosity=values[84]; d.shear_factor=values[85]; d.transverse_shear_modulus=values[86];
  d.translational_stiffness=values[87]; d.rotational_stiffness=values[88];
  d.unscaled_element_dt=values[89]; d.internal_work_increment={values[90],values[91]};
  if(d.effective_thickness!=p.thickness||d.native_sound_speed<=0||d.membrane_viscosity<0||
     d.shear_factor<=0||d.transverse_shear_modulus<=0||d.translational_stiffness<=0||
     d.rotational_stiffness<=0||d.unscaled_element_dt<=0) return Status::kNonfiniteResult;
  output=next; return Status::kSuccess;
}
} // namespace tl::qualification::t3
