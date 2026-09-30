// SPDX-License-Identifier: AGPL-3.0-or-later
// Explicit boundary around the unchanged native CZCORP5 projection algebra.
#pragma once
#include "QephProjection.h"
namespace tl::fea::qeph::detail {
namespace projection_units {
TL_QEPH_HD inline bool Converted(double before,double after) {
  return tl::math::Finite(before)&&tl::math::Finite(after)&&(before==0||after!=0);
}
TL_QEPH_HD inline bool Divide(double& value,double scale) {
  const double next=value/scale;
  if(!Converted(value,next))return false;
  value=next;return true;
}
TL_QEPH_HD inline bool Multiply(double& value,double scale) {
  const double next=value*scale;
  if(!Converted(value,next))return false;
  value=next;return true;
}
TL_QEPH_HD inline bool Divide(Vec3& value,double scale) {
  return Divide(value.x,scale)&&Divide(value.y,scale)&&Divide(value.z,scale);
}
TL_QEPH_HD inline bool Multiply(Vec3& value,double scale) {
  return Multiply(value.x,scale)&&Multiply(value.y,scale)&&Multiply(value.z,scale);
}
// Only fields actually read by CZCORP5 are reexpressed. The material/rate
// workspace outside this private packet retains its original SI units.
TL_QEPH_HD inline bool RatePacket(GeometryWork& work,double length) {
  const double area=length*length;
  auto& k=work.values;
  if(!Divide(k.area,area)||!Multiply(k.reciprocal_area,area)||
      !Divide(k.effective_warpage,length)||!Divide(work.l13,area)||
      !Divide(work.l24,area)||!Divide(work.lm,area))return false;
  if(!Divide(work.x13,length)||!Divide(work.x24,length)||
      !Divide(work.y13,length)||!Divide(work.y24,length)||
      !Divide(work.mx13,length)||!Divide(work.my13,length))return false;
  for(unsigned i=0;i<4;++i)
    if(!Divide(k.local_position[i].x,length)||!Divide(k.local_position[i].y,length))return false;
  return Divide(work.v13,length)&&Divide(work.v24,length)&&Divide(work.vhi,length);
}
TL_QEPH_HD inline bool Matrices(const Kinematics& k) {
  for(double value:k.projection_inverse)if(!tl::math::Finite(value))return false;
  for(unsigned n=0;n<4;++n)
    if(!Finite(k.local_normals[n])||!Finite(k.projection_columns[n]))return false;
  for(double value:k.projected_omega)if(!tl::math::Finite(value))return false;
  return true;
}
} // namespace projection_units

// Input is the coherent SI CurrentGeometry/Gather/CorrectMidpoint packet.
// Time stays seconds; x and v divide by length, omega is unchanged. DI/DB are
// retained exactly in that working metric and tagged, never scaled and rounded
// back. Public positions, areas and projected velocities remain SI.
TL_QEPH_HD inline Status ProjectWarpedRatesInWorkingLength(const PrescribedInterval& interval,
    double length,GeometryWork& output) {
  if(!ValidProjectionLength(length))return Status::kInvalidReference;
  if(length==1.) {
    // Original arithmetic/order is the exact legacy path. PrepareGeometry owns
    // this private candidate and its existing public failure/publication fence.
    const auto status=ProjectWarpedRates(interval,output);
    if(status==Status::kSuccess)output.values.projection_metric.working_length_m=1.;
    return status;
  }
  auto work=output;
  if(!projection_units::RatePacket(work,length))return Status::kNonfiniteResult;
  const auto status=ProjectWarpedRates(interval,work);
  if(status!=Status::kSuccess)return status;
  if(!projection_units::Multiply(work.v13,length)||!projection_units::Multiply(work.v24,length)||
      !projection_units::Multiply(work.vhi,length)||!projection_units::Matrices(work.values))
    return Status::kNonfiniteResult;
  // Geometry itself never changed physically. Source only clears Z1 on the
  // planar branch, so retain the original SI value on the warped branch rather
  // than introduce an unnecessary length divide/multiply round trip.
  const double effective=work.values.planar?0.:output.values.effective_warpage;
  // Every rejection is above this fixed-size, nonthrowing publication. There
  // is no second retained matrix cache or second full GeometryWork copy.
  output.v13=work.v13;output.v24=work.v24;output.vhi=work.vhi;
  output.values.planar=work.values.planar;
  output.values.effective_warpage=effective;
  for(unsigned n=0;n<4;++n) {
    output.values.local_normals[n]=work.values.local_normals[n];
    output.values.projection_columns[n]=work.values.projection_columns[n];
  }
  for(unsigned i=0;i<6;++i)output.values.projection_inverse[i]=work.values.projection_inverse[i];
  for(unsigned i=0;i<8;++i)output.values.projected_omega[i]=work.values.projected_omega[i];
  output.values.projection_metric.working_length_m=length;
  return Status::kSuccess;
}
} // namespace tl::fea::qeph::detail
