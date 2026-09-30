// SPDX-License-Identifier: AGPL-3.0-or-later
// OpenRadioss (C) 2026 Siemens, complete CZPROJN IFINI0/IMPL_S0/IDRIL0 path.
#pragma once
#include "QephElasticForces.h"

namespace tl::fea::qeph::detail {
TL_QEPH_HD inline Vec3 WorldForce(const Matrix3& q,Vec3 x) {
  return {q.v[0]*x.x+q.v[1]*x.y+q.v[2]*x.z,
          q.v[3]*x.x+q.v[4]*x.y+q.v[5]*x.z,
          q.v[6]*x.x+q.v[7]*x.y+q.v[8]*x.z};
}
TL_QEPH_HD inline void ProjectForces(const GeometryWork& g,const LocalForceWork& w,
                                     Vec3 (&force)[4],Vec3 (&couple)[4]) {
  Vec3 fl[4]{},ml[4]{},mm[4]{};
  for(unsigned n=0;n<4;++n) {
    const unsigned a=n%2,s=2+a;
    const double sign=n<2?1.:-1.;
    // Unpack native symmetric/antisymmetric slots; IFINI0 has no FZERO add.
    fl[n]={sign*w.force[a].x+w.force[s].x,sign*w.force[a].y+w.force[s].y,
           sign*w.force[a].z+w.force[s].z};
    ml[n]={sign*w.couple[a][0]+w.couple[s][0],sign*w.couple[a][1]+w.couple[s][1],0};
  }
  const auto& k=g.values; const auto* x=k.local_position;
  if(k.planar) {
    for(unsigned n=0;n<4;++n) {
      force[n]=WorldForce(k.frame,fl[n]);
      // Preserve native two-term sum instead of adding an artificial z term.
      couple[n]={k.frame.v[0]*ml[n].x+k.frame.v[1]*ml[n].y,
                 k.frame.v[3]*ml[n].x+k.frame.v[4]*ml[n].y,
                 k.frame.v[6]*ml[n].x+k.frame.v[7]*ml[n].y};
    }
    return;
  }
  const double z=k.effective_warpage; const auto* normals=k.local_normals;
  const auto* db=k.projection_columns; const auto* di=k.projection_inverse;
  const Vec3 ar{
      -z*(fl[0].y-fl[1].y+fl[2].y-fl[3].y)+x[0].y*fl[0].z+ml[0].x+
       x[1].y*fl[1].z+ml[1].x+x[2].y*fl[2].z+ml[2].x+x[3].y*fl[3].z+ml[3].x,
      z*(fl[0].x-fl[1].x+fl[2].x-fl[3].x)-x[0].x*fl[0].z+ml[0].y-
       x[1].x*fl[1].z+ml[1].y-x[2].x*fl[2].z+ml[2].y-x[3].x*fl[3].z+ml[3].y,
      -x[0].y*fl[0].x+x[0].x*fl[0].y-x[1].y*fl[1].x+x[1].x*fl[1].y-
       x[2].y*fl[2].x+x[2].x*fl[2].y-x[3].y*fl[3].x+x[3].x*fl[3].y};
  double ad[4]{},ald[4]{};
  for(unsigned n=0;n<4;++n) ad[n]=normals[n].x*ml[n].x+normals[n].y*ml[n].y;
  const Vec3 dbad{db[0].x*ad[0]+db[1].x*ad[1]+db[2].x*ad[2]+db[3].x*ad[3],
                 db[0].y*ad[0]+db[1].y*ad[1]+db[2].y*ad[2]+db[3].y*ad[3],
                 db[0].z*ad[0]+db[1].z*ad[1]+db[2].z*ad[2]+db[3].z*ad[3]};
  const Vec3 alr{di[0]*ar.x+di[3]*ar.y+di[4]*ar.z-dbad.x,
                 di[3]*ar.x+di[1]*ar.y+di[5]*ar.z-dbad.y,
                 di[4]*ar.x+di[5]*ar.y+di[2]*ar.z-dbad.z};
  for(unsigned n=0;n<4;++n)
    ald[n]=ad[n]+normals[n].x*dbad.x+normals[n].y*dbad.y+normals[n].z*dbad.z-
        db[n].x*ar.x-db[n].y*ar.y-db[n].z*ar.z;
  const double cx=z*alr.y,cy=z*alr.x;
  for(unsigned n=0;n<4;++n) {
    if(n%2==0) {
      fl[n].x=fl[n].x-cx+x[n].y*alr.z;
      fl[n].y=fl[n].y+cy-x[n].x*alr.z;
    } else {
      fl[n].x=fl[n].x+cx+x[n].y*alr.z;
      fl[n].y=fl[n].y-cy-x[n].x*alr.z;
    }
    fl[n].z=fl[n].z-x[n].y*alr.x+x[n].x*alr.y;
    mm[n]={ml[n].x-alr.x-normals[n].x*ald[n],
           ml[n].y-alr.y-normals[n].y*ald[n],-alr.z-normals[n].z*ald[n]};
    force[n]=WorldForce(k.frame,fl[n]); couple[n]=WorldForce(k.frame,mm[n]);
  }
}
} // namespace tl::fea::qeph::detail
