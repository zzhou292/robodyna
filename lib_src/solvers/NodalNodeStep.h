#pragma once
#include "FENodalStateStorage.h"
#include "NodalRotation.h"

namespace tl::fea::nodal_detail {
// Shared unchanged TL orientation operation for ordinary and rigid member
// nodes. A native principal-frame update is a separate group operation.
TL_SURFACE_HD inline NodalStatus PrepareNodeOrientation(const double* accepted,double* trial,
    std::uint32_t i,std::uint32_t n,double h,double maximum_angle,bool fixed_rotation) {
  double increment[3];
  for(unsigned a=0;a<3;++a) increment[a]=h*trial[6*n+3*i+a];
  const double angle=::hypot(::hypot(increment[0],increment[1]),increment[2]);
  if(!tlfea::contact::IsFinite(angle)||angle>maximum_angle) return NodalStatus::StepTooLarge;
  const auto initial=ReadQuaternion(accepted+9*n+4*i);
  tl::math::Quaternion candidate;
  if(fixed_rotation) {
    if(!UnitQuaternion(initial)) return NodalStatus::InvalidOutput;
    candidate=initial;
  } else if(!IncrementWorldRotation(initial,increment,candidate)) return NodalStatus::InvalidOutput;
  auto* q=trial+9*n+4*i;
  q[0]=candidate.w; q[1]=candidate.x; q[2]=candidate.y; q[3]=candidate.z;
  return NodalStatus::Ok;
}

template<bool Capture=false>
TL_SURFACE_HD inline NodalStatus AdvanceOrdinaryNode(const double* accepted,double* trial,
    const double* force,const double* inverse,const std::uint8_t* constraints,
    std::uint32_t i,std::uint32_t n,double h,double kick_dt,double maximum_angle,
    double* acceleration_xyz=nullptr,double* angular_acceleration_xyz=nullptr) {
  if(!AdvanceTranslationNodeWithKick<Capture>(accepted,trial,force,inverse[i],constraints[n+i],i,n,h,kick_dt,trial+13*n,acceleration_xyz))
    return NodalStatus::InvalidOutput;
  const bool fixed_rotation=constraints[2*n+i]!=0;
  for(unsigned a=0;a<3;++a) {
    const auto j=3*i+a;
    const double couple=force[(3+a)*n+i];
    const double acceleration=fixed_rotation?0:inverse[n+i]*couple;
    const double omega=fixed_rotation?0:accepted[6*n+j]+kick_dt*acceleration;
    trial[6*n+j]=omega; trial[16*n+j]=fixed_rotation?-couple:0;
    if constexpr(Capture) angular_acceleration_xyz[j]=acceleration;
    const double increment=h*omega;
    if(!tlfea::contact::IsFinite(acceleration)||!tlfea::contact::IsFinite(omega)||!tlfea::contact::IsFinite(increment))
      return NodalStatus::InvalidOutput;
  }
  return PrepareNodeOrientation(accepted,trial,i,n,h,maximum_angle,fixed_rotation);
}
} // namespace tl::fea::nodal_detail
