// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected complete C3DEFO3/C3CURV3 arithmetic, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "T3CurrentGeometry.h"
namespace tl::fea::t3::detail {
TL_T3_HD inline void DeformationRates(const PrescribedInterval& in,GeometryWork& work) {
  auto& k=work.kinematics; const auto& f=k.frame;
  double vx[3],vy[3],vz[3];
  for(unsigned n=0;n<3;++n) {
    vx[n]=Project(f,0,in.velocity[n]); vy[n]=Project(f,1,in.velocity[n]); vz[n]=Project(f,2,in.velocity[n]);
  }
  const double px=k.derivative[0],py1=k.derivative[1],py2=k.derivative[2];
  const double dtv4=.25*in.dt,dtv4b=dtv4; // Explicit IMPL_S0 / ISH3N2.
  const double vz12=vz[0]-vz[1],vz13=vz[0]-vz[2],vz23=vz[1]-vz[2];
  const double tmp1=dtv4*vz12/(py1+py2);
  double tmp2=(py1*vz[0]+py2*vz[1])/(py1+py2);
  tmp2=dtv4*(tmp2-vz[2])/px;
  double vy12=vy[0]-vy[1];
  const double tmp11=dtv4b*vy12/(py1+py2);
  double tmp22=(py1*vx[0]+py2*vx[1])/(py1+py2);
  tmp22=dtv4b*(tmp22-vx[2])/px;
  const double vx10=vx[0],vx20=vx[1],vx30=vx[2];
  vx[0]=vx[0]-vz[0]*tmp1-vy[0]*tmp11;
  vx[1]=vx[1]-vz[1]*tmp1-vy[1]*tmp11;
  vx[2]=vx[2]-vz[2]*tmp1-vy[2]*tmp11;
  vy[0]=vy[0]-vz[0]*tmp2-vx10*tmp22;
  vy[1]=vy[1]-vz[1]*tmp2-vx20*tmp22;
  vy[2]=vy[2]-vz[2]*tmp2-vx30*tmp22;
  const double vx12=vx[0]-vx[1],vx13=vx[0]-vx[2],vy13=vy[0]-vy[2];
  const double vx23=vx[1]-vx[2],vy23=vy[1]-vy[2]; vy12=vy[0]-vy[1];
  k.raw_rate[0]=px*vx12;
  k.raw_rate[1]=py1*vy13+py2*vy23;
  k.raw_rate[2]=py1*vx13+py2*vx23+px*vy12;
  k.raw_rate[3]=py1*vz13+py2*vz23;
  k.raw_rate[4]=px*vz12;
  k.corrected_velocity_difference[0]=vx13; k.corrected_velocity_difference[1]=vx23;
  k.corrected_velocity_difference[2]=vy12;
}
TL_T3_HD inline void CurvatureRates(const PrescribedInterval& in,GeometryWork& work) {
  auto& k=work.kinematics; const auto& f=k.frame; double rx[3],ry[3];
  for(unsigned n=0;n<3;++n) { rx[n]=Project(f,0,in.angular_velocity[n]); ry[n]=Project(f,1,in.angular_velocity[n]); }
  const double px=k.derivative[0],py1=k.derivative[1],py2=k.derivative[2];
  const double rx12=rx[0]-rx[1],rx13=rx[0]-rx[2],rx23=rx[1]-rx[2];
  k.raw_rate[6]=-py1*rx13-py2*rx23;
  k.raw_rate[7]=px*rx12;
  const double ry12=ry[0]-ry[1],ry13=ry[0]-ry[2],ry23=ry[1]-ry[2];
  k.raw_rate[5]=px*ry12;
  k.raw_rate[7]=py1*ry13+py2*ry23-k.raw_rate[7];
  const double ryav=px*(px*(-rx[0]+rx[1])
    +(2*py1+3*py2)*ry[0]+(3*py1+2*py2)*ry[1]+(py1+py2)*ry[2]);
  const double rxav=-px*((2*py1+py2)*rx[0]+(py1+2*py2)*rx[1]+3*(py1+py2)*rx[2])
    +py1*(py1+2*py2)*ry[0]-py2*(2*py1+py2)*ry[1]+(py2*py2-py1*py1)*ry[2];
  k.raw_rate[4]=k.raw_rate[4]+ryav*(1./3);
  k.raw_rate[3]=k.raw_rate[3]+rxav*(1./3);
  // ISMSTR=-1: the source's ISMSTR10 WXY branch is inactive.
}
TL_T3_HD inline Status EvaluateRates(const PrescribedInterval& in,GeometryWork& work) {
  DeformationRates(in,work); CurvatureRates(in,work);
  auto& k=work.kinematics;
  for(unsigned i=0;i<8;++i) {
    k.normalized_rate[i]=k.raw_rate[i]/k.area;
    if(!tl::math::Finite(k.raw_rate[i])||!tl::math::Finite(k.normalized_rate[i])) return Status::kNonfiniteResult;
  }
  for(double x:k.corrected_velocity_difference) if(!tl::math::Finite(x)) return Status::kNonfiniteResult;
  return Status::kSuccess;
}
} // namespace tl::fea::t3::detail
