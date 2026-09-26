// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Device.h"
#include "Packing.h"
namespace tlfea::contact::radioss_type25::candidates::detail {
__device__ inline Vector Read(Device d,VectorView view,std::uint32_t i,bool velocity=false) {
  const auto v=view.at(i);const double divisor=d.si?(velocity?d.velocity:d.length):1.;
  return {v.x/divisor,v.y/divisor,v.z/divisor};
}
__device__ inline double Gap(Device d,double x){return d.si?x/d.length:x;}
// Shared unchanged Engine main admission/ScreenBounds packing. Inactive STF
// precedes every geometry/gap read; lack of active secondaries follows those
// validations, preserving the original axis-sweep consumed-field boundary.
__device__ inline Status MainEnvelope(Device d,const Current& in,std::size_t i,
    Envelope& envelope,bool& enabled) {
  enabled=false;const auto m=d.mains[i].source;const double stiffness=in.main_stiffness[i];
  if(!(d.main_coefficient_domain==MainCoefficientDomain::NativeSigned?
       tl::math::Finite(stiffness):Nonnegative(stiffness)))return Status::InvalidInput;
  if(stiffness<=0.)return Status::Ok;
  bool valid=Nonnegative(Gap(d,in.main_gaps[i]))&&Nonnegative(Gap(d,in.main_curvature[i]));
  ScreenRow row;row.margin=in.margin;row.curvature=Gap(d,in.main_curvature[i]);row.main_gap=Gap(d,in.main_gaps[i]);
  row.secondary_gap=__longlong_as_double(static_cast<long long>(d.control->maximum_gap_bits));
  row.gap_load=in.gap_load;row.drad=in.drad;row.stored_motion=in.stored_motion;
  for(unsigned j=0;j<4;++j) {
    row.vertices[j]=Read(d,in.positions,m.nodes[j]);valid=valid&&tl::math::fixed3::Finite(row.vertices[j]);
  }
  if(!valid)return Status::InvalidInput;
  if(!d.control->active)return Status::Ok;
  const auto status=ScreenBounds(row,&envelope);enabled=status==Status::Ok;return status;
}
}
