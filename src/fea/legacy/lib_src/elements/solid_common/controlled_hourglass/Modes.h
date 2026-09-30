// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected SHOUR_CTL, OpenRadioss (C) 2026 Siemens, revision a62b27e6.
#pragma once
#include "Types.h"

namespace tl::fea::solid_common::controlled_hourglass::detail {
TL_BRICK_HD inline void Modes(const Input& input,double (&g)[8][3],
    double (&rate)[3][4]) noexcept {
  const auto& p=input.projection;
  g[0][0]= 1.0/8.0-p[0][0];g[1][0]= 1.0/8.0-p[1][0];
  g[2][0]=-1.0/8.0-p[2][0];g[3][0]=-1.0/8.0-p[3][0];
  g[4][0]=-1.0/8.0+p[2][0];g[5][0]=-1.0/8.0+p[3][0];
  g[6][0]= 1.0/8.0+p[0][0];g[7][0]= 1.0/8.0+p[1][0];
  g[0][1]= 1.0/8.0-p[0][1];g[1][1]=-1.0/8.0-p[1][1];
  g[2][1]=-1.0/8.0-p[2][1];g[3][1]= 1.0/8.0-p[3][1];
  g[4][1]=-1.0/8.0+p[2][1];g[5][1]= 1.0/8.0+p[3][1];
  g[6][1]= 1.0/8.0+p[0][1];g[7][1]=-1.0/8.0+p[1][1];
  g[0][2]= 1.0/8.0-p[0][2];g[1][2]=-1.0/8.0-p[1][2];
  g[2][2]= 1.0/8.0-p[2][2];g[3][2]=-1.0/8.0-p[3][2];
  g[4][2]= 1.0/8.0+p[2][2];g[5][2]=-1.0/8.0+p[3][2];
  g[6][2]= 1.0/8.0+p[0][2];g[7][2]=-1.0/8.0+p[1][2];
  for(unsigned k=0;k<3;++k) {
    double v[8];for(unsigned n=0;n<8;++n)v[n]=Component(input.local_velocity_m_s[n],k);
    for(unsigned h=0;h<3;++h)
      rate[k][h]=g[0][h]*v[0]+g[1][h]*v[1]+g[2][h]*v[2]+g[3][h]*v[3]+
                 g[4][h]*v[4]+g[5][h]*v[5]+g[6][h]*v[6]+g[7][h]*v[7];
    rate[k][3]=(1.0/64.0)*(v[0]-v[1]+v[2]-v[3]-v[4]+v[5]-v[6]+v[7]);
  }
}
TL_BRICK_HD inline void Forces(const Input& input,const double (&g)[8][3],
    const double (&mode)[3][4],Vec3 (&force)[8]) noexcept {
  for(unsigned k=0;k<3;++k) {
    const auto& h=mode[k];
    // Preserve each native subtraction/addition, including incoming force.
    SetComponent(force[0],k,Component(input.incoming_local_force_n[0],k)-g[0][0]*h[0]-g[0][1]*h[1]-g[0][2]*h[2]-h[3]);
    SetComponent(force[1],k,Component(input.incoming_local_force_n[1],k)-g[1][0]*h[0]-g[1][1]*h[1]-g[1][2]*h[2]+h[3]);
    SetComponent(force[2],k,Component(input.incoming_local_force_n[2],k)-g[2][0]*h[0]-g[2][1]*h[1]-g[2][2]*h[2]-h[3]);
    SetComponent(force[3],k,Component(input.incoming_local_force_n[3],k)-g[3][0]*h[0]-g[3][1]*h[1]-g[3][2]*h[2]+h[3]);
    SetComponent(force[4],k,Component(input.incoming_local_force_n[4],k)-g[4][0]*h[0]-g[4][1]*h[1]-g[4][2]*h[2]+h[3]);
    SetComponent(force[5],k,Component(input.incoming_local_force_n[5],k)-g[5][0]*h[0]-g[5][1]*h[1]-g[5][2]*h[2]-h[3]);
    SetComponent(force[6],k,Component(input.incoming_local_force_n[6],k)-g[6][0]*h[0]-g[6][1]*h[1]-g[6][2]*h[2]+h[3]);
    SetComponent(force[7],k,Component(input.incoming_local_force_n[7],k)-g[7][0]*h[0]-g[7][1]*h[1]-g[7][2]*h[2]-h[3]);
  }
}
} // namespace tl::fea::solid_common::controlled_hourglass::detail
