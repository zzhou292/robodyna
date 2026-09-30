// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Packet.h"
#include "lib_src/elements/qeph/QephKinematics.h"
#include "lib_src/elements/qeph/QephForceProjection.h"
#include <stdexcept>
#include <initializer_list>
namespace qeph_projection_test {
namespace q=tl::fea::qeph;
TL_QEPH_HD inline q::Vec3 Vector(V3 x){return {x[0],x[1],x[2]};}
TL_QEPH_HD inline V3 Vector(q::Vec3 x){return {x.x,x.y,x.z};}
TL_QEPH_HD inline q::detail::GeometryWork Work(const Geometry& in) {
  q::detail::GeometryWork out;
  out.values.area=in.area;out.values.reciprocal_area=in.area_i;
  out.x13=in.x13;out.x24=in.x24;out.y13=in.y13;out.y24=in.y24;
  out.mx13=in.mx13;out.my13=in.my13;out.values.effective_warpage=in.z1;
  out.lm=in.ll;out.l13=in.l13;out.l24=in.l24;
  for(unsigned i=0;i<9;++i)out.values.frame.v[i]=in.vq[i];
  for(unsigned i=0;i<4;++i)out.values.local_position[i]={in.corel[i][0],in.corel[i][1],0};
  return out;
}
inline Geometry GeometryOf(const q::detail::GeometryWork& in) {
  Geometry out;out.area=in.values.area;out.area_i=in.values.reciprocal_area;
  out.x13=in.x13;out.x24=in.x24;out.y13=in.y13;out.y24=in.y24;out.mx13=in.mx13;out.my13=in.my13;
  out.z1=in.values.effective_warpage;out.ll=in.lm;out.l13=in.l13;out.l24=in.l24;
  for(unsigned i=0;i<9;++i)out.vq[i]=in.values.frame.v[i];
  for(unsigned i=0;i<4;++i)out.corel[i]={in.values.local_position[i].x,in.values.local_position[i].y};return out;
}
TL_QEPH_HD inline RateResult RateResultOf(const q::detail::GeometryWork& work) {
  RateResult out;const auto& k=work.values;out.projection.planar=k.planar;out.projection.z1=k.effective_warpage;
  out.projection.warped_defined=!k.planar;out.v13=Vector(work.v13);out.v24=Vector(work.v24);out.vhi=Vector(work.vhi);
  for(unsigned i=0;i<4;++i){out.rlxyz[i]={k.projected_omega[2*i],k.projected_omega[2*i+1]};
    if(!k.planar){out.projection.db[i]=Vector(k.projection_columns[i]);out.projection.vqn[i]=Vector(k.local_normals[i]);}}
  if(!k.planar)for(unsigned i=0;i<6;++i)out.projection.di[i]=k.projection_inverse[i];return out;
}
inline RateResult CppRates(const RateInput& in) {
  auto work=Work(in.geometry);work.v13=Vector(in.v13);work.v24=Vector(in.v24);work.vhi=Vector(in.vhi);
  q::PrescribedInterval interval;
  for(unsigned i=0;i<4;++i){interval.omega_midpoint[i]=Vector(in.world_omega[i]);
    work.values.projected_omega[2*i]=in.rlxyz[i][0];work.values.projected_omega[2*i+1]=in.rlxyz[i][1];}
  if(q::detail::ProjectWarpedRates(interval,work)!=q::Status::kSuccess)throw std::runtime_error("C++ raw projection failed");
  return RateResultOf(work);
}
inline ForceResult CppForces(const ForceInput& in) {
  auto work=Work(in.geometry);auto& k=work.values;k.planar=in.projection.planar;k.effective_warpage=in.projection.z1;
  if(!k.planar){if(!in.projection.warped_defined)throw std::runtime_error("Undefined warped input");
    for(unsigned i=0;i<6;++i)k.projection_inverse[i]=in.projection.di[i];
    for(unsigned i=0;i<4;++i){k.projection_columns[i]=Vector(in.projection.db[i]);k.local_normals[i]=Vector(in.projection.vqn[i]);}}
  q::detail::LocalForceWork local;for(unsigned i=0;i<4;++i){local.force[i]=Vector(in.vf[i]);
    local.couple[i][0]=in.vm[i][0];local.couple[i][1]=in.vm[i][1];}
  q::Vec3 force[4],couple[4];q::detail::ProjectForces(work,local,force,couple);ForceResult out;
  for(unsigned i=0;i<4;++i){out.force[i]=Vector(force[i]);out.couple[i]=Vector(couple[i]);}return out;
}
// Same physical raw operands in another length unit. Projection matrices are
// deliberately NOT scaled; recompute them from the new metric with NativeRates.
inline Geometry Scale(Geometry in,double ratio) {
  const double square=ratio*ratio;in.area*=square;in.area_i/=square;
  in.x13*=ratio;in.x24*=ratio;in.y13*=ratio;in.y24*=ratio;in.mx13*=ratio;in.my13*=ratio;in.z1*=ratio;
  in.ll*=square;in.l13*=square;in.l24*=square;for(auto& x:in.corel)for(auto& v:x)v*=ratio;return in;
}
inline RateInput Scale(RateInput in,double ratio) {
  in.geometry=Scale(in.geometry,ratio);for(auto* v:{&in.v13,&in.v24,&in.vhi})for(auto& x:*v)x*=ratio;return in;
}
inline ForceInput Scale(ForceInput in,double ratio,Projection projection) {
  in.geometry=Scale(in.geometry,ratio);in.projection=projection;for(auto& moment:in.vm)for(auto& x:moment)x*=ratio;return in;
}
inline RateInput CallerRates(const CapturedRow& row,double ratio) {
  q::PrescribedInterval interval;
  for(unsigned i=0;i<4;++i) {
    interval.position_endpoint[i]=Vector(row.position[i]);interval.velocity_midpoint[i]=Vector(row.velocity[i]);
    interval.omega_midpoint[i]=Vector(row.omega[i]);
    interval.position_endpoint[i].x*=ratio;interval.position_endpoint[i].y*=ratio;interval.position_endpoint[i].z*=ratio;
    interval.velocity_midpoint[i].x*=ratio;interval.velocity_midpoint[i].y*=ratio;interval.velocity_midpoint[i].z*=ratio;
  }
  q::detail::GeometryWork work;
  if(q::detail::CurrentGeometry(interval,work)!=q::Status::kSuccess)throw std::runtime_error("Captured caller geometry rejected");
  q::detail::GatherRates(interval,work);q::detail::CorrectMidpointVelocity(row.dt1,work);
  RateInput out;out.controls=row.rate_entry.controls;out.geometry=GeometryOf(work);
  out.v13=Vector(work.v13);out.v24=Vector(work.v24);out.vhi=Vector(work.vhi);
  for(unsigned i=0;i<4;++i){out.world_omega[i]=row.omega[i];out.rlxyz[i]={work.values.projected_omega[2*i],work.values.projected_omega[2*i+1]};}
  return out;
}
} // namespace qeph_projection_test
