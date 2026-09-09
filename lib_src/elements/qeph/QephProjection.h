// SPDX-License-Identifier: AGPL-3.0-or-later
// Adapted from OpenRadioss, Copyright (C) 2026 Siemens.
// CZCORP5 full projection, IMPL_S0/IRESP2/IDRIL0/NPT0. No drilling-only branch.
#pragma once
#include "QephProjectionInverse.h"

namespace tl::fea::qeph::detail {
TL_QEPH_HD inline bool WarpedNormals(GeometryWork& g) {
  auto& o=g.values; auto& n=o.local_normals;
  const double z=o.effective_warpage,z2=z*z,a4=o.area*.25;
  double sz1=g.mx13*g.y24-g.my13*g.x24,sz2=a4+sz1,sz=z2*g.l24;
  double sl=1/::sqrt(sz+sz2*sz2);
  n[0]={-z*g.y24,z*g.x24,sz2*sl}; n[2].x=-n[0].x; n[2].y=-n[0].y;
  n[0].x=n[0].x*sl; n[0].y=n[0].y*sl;
  sz2=a4-sz1; sl=1/::sqrt(sz+sz2*sz2);
  n[2].x=n[2].x*sl; n[2].y=n[2].y*sl; n[2].z=sz2*sl;
  sz1=g.mx13*g.y13-g.my13*g.x13; sz2=a4+sz1; sz=z2*g.l13;
  sl=1/::sqrt(sz+sz2*sz2);
  n[1]={-z*g.y13,z*g.x13,sz2*sl}; n[3].x=-n[1].x; n[3].y=-n[1].y;
  n[1].x=n[1].x*sl; n[1].y=n[1].y*sl;
  sz2=a4-sz1; sl=1/::sqrt(sz+sz2*sz2);
  n[3].x=n[3].x*sl; n[3].y=n[3].y*sl; n[3].z=sz2*sl;
  for(const auto& value:n) if(!Finite(value)) return false;
  return true;
}
TL_QEPH_HD inline bool PrepareProjectionInverse(GeometryWork& g) {
  auto& o=g.values; const auto& p=o.local_position; const auto& n=o.local_normals;
  const double xx=FourProducts(p,p,0,0),yy=FourProducts(p,p,1,1),xy=FourProducts(p,p,0,1);
  const double xz=(p[0].x-p[1].x+p[2].x-p[3].x)*o.effective_warpage;
  const double yz=(p[0].y-p[1].y+p[2].y-p[3].y)*o.effective_warpage;
  const double z2=o.effective_warpage*o.effective_warpage,zz=4*z2;
  const double btb[]{FourProducts(n,n,0,0),FourProducts(n,n,1,1),FourProducts(n,n,2,2),
                     FourProducts(n,n,0,1),FourProducts(n,n,0,2),FourProducts(n,n,1,2)};
  const double d[]{yy+zz+4-btb[0],xx+zz+4-btb[1],xx+yy+4-btb[2],
                   -xy-btb[3],-xz-btb[4],-yz-btb[5]};
  if(!ProjectionInverse(d,o.projection_inverse)) return false;
  const auto& di=o.projection_inverse;
  for(unsigned j=0;j<4;++j) {
    o.projection_columns[j]={di[0]*n[j].x+di[3]*n[j].y+di[4]*n[j].z,
                             di[3]*n[j].x+di[1]*n[j].y+di[5]*n[j].z,
                             di[4]*n[j].x+di[5]*n[j].y+di[2]*n[j].z};
    if(!Finite(o.projection_columns[j])) return false;
  }
  return true;
}
TL_QEPH_HD inline Status ProjectWarpedRates(const PrescribedInterval& interval,GeometryWork& g) {
  auto& o=g.values;
  const double z2=o.effective_warpage*o.effective_warpage;
  if(!tl::math::Finite(z2)) return Status::kNonfiniteResult;
  o.planar=z2<g.lm*1e-8; // Native IRESP2 TOL and NPT0; observable source switch.
  if(o.planar) {
    o.effective_warpage=0;
    for(auto& n:o.local_normals) n={0,0,1}; // Native public diagnostic convention.
    return Status::kSuccess;
  }
  if(!WarpedNormals(g)) return Status::kNonfiniteResult;
  const auto& n=o.local_normals; const auto& db=o.projection_columns;
  Vec3 rr[4]; double ad[4];
  for(unsigned j=0;j<4;++j) {
    rr[j]={o.projected_omega[2*j],o.projected_omega[2*j+1],Project(o.frame,interval.omega_midpoint[j]).z};
    ad[j]=Dot(n[j],rr[j]);
  }
  const double z=o.effective_warpage;
  const Vec3 ar{-z*g.vhi.y+g.y13*g.v13.z+g.y24*g.v24.z+g.my13*g.vhi.z+rr[0].x+rr[1].x+rr[2].x+rr[3].x,
                 z*g.vhi.x-g.x13*g.v13.z-g.x24*g.v24.z-g.mx13*g.vhi.z+rr[0].y+rr[1].y+rr[2].y+rr[3].y,
                 g.x13*g.v13.y+g.x24*g.v24.y+g.mx13*g.vhi.y-g.y13*g.v13.x-g.y24*g.v24.x-g.my13*g.vhi.x+
                 rr[0].z+rr[1].z+rr[2].z+rr[3].z};
  if(!PrepareProjectionInverse(g)) return Status::kNonfiniteResult;
  const Vec3 dbad{db[0].x*ad[0]+db[1].x*ad[1]+db[2].x*ad[2]+db[3].x*ad[3],
                   db[0].y*ad[0]+db[1].y*ad[1]+db[2].y*ad[2]+db[3].y*ad[3],
                   db[0].z*ad[0]+db[1].z*ad[1]+db[2].z*ad[2]+db[3].z*ad[3]};
  const auto& di=o.projection_inverse;
  const Vec3 alr{di[0]*ar.x+di[3]*ar.y+di[4]*ar.z-dbad.x,
                  di[3]*ar.x+di[1]*ar.y+di[5]*ar.z-dbad.y,
                  di[4]*ar.x+di[5]*ar.y+di[2]*ar.z-dbad.z};
  double ald[4];
  for(unsigned j=0;j<4;++j)
    ald[j]=ad[j]+n[j].x*dbad.x+n[j].y*dbad.y+n[j].z*dbad.z-db[j].x*ar.x-db[j].y*ar.y-db[j].z*ar.z;
  const double c1=2*alr.z;
  g.v13.x=g.v13.x+c1*g.y13; g.v24.x=g.v24.x+c1*g.y24;
  g.vhi.x=g.vhi.x+4*(alr.z*g.my13-z*alr.y);
  g.v13.y=g.v13.y-c1*g.x13; g.v24.y=g.v24.y-c1*g.x24;
  g.vhi.y=g.vhi.y-4*(alr.z*g.mx13-z*alr.x);
  g.v13.z=g.v13.z-2*(g.y13*alr.x-g.x13*alr.y);
  g.v24.z=g.v24.z-2*(g.y24*alr.x-g.x24*alr.y);
  g.vhi.z=g.vhi.z+4*(g.mx13*alr.y-g.my13*alr.x);
  for(unsigned j=0;j<4;++j) {
    o.projected_omega[2*j]=rr[j].x-alr.x-n[j].x*ald[j];
    o.projected_omega[2*j+1]=rr[j].y-alr.y-n[j].y*ald[j];
  }
  return Status::kSuccess;
}
} // namespace tl::fea::qeph::detail
