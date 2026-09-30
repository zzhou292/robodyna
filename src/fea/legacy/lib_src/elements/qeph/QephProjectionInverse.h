// SPDX-License-Identifier: AGPL-3.0-or-later
// Adapted from OpenRadioss, Copyright (C) 2026 Siemens.
// Complete selected IRESP2 inverse expressions inside CZCORP5; not a general
// matrix inverse or a different solver. The source floors remain unchanged.
#pragma once
#include "QephGeometryWork.h"

namespace tl::fea::qeph::detail {
TL_QEPH_HD inline bool ProjectionInverse(const double (&d)[6],double (&di)[6]) {
  for(double value:d) if(!tl::math::Finite(value)) return false;
  const double abc=d[0]*d[1]*d[2],xxyz2=d[0]*d[5]*d[5];
  const double yyxz2=d[1]*d[4]*d[4],zzxy2=d[2]*d[3]*d[3];
  double deta=::fabs(abc+2*d[3]*d[4]*d[5]-xxyz2-yyxz2-zzxy2);
  if (!tl::math::Finite(deta)) return false;
  deta=1/::fmax(deta,1e-20);
  di[0]=(abc-xxyz2)*deta/::fmax(d[0],1e-20);
  di[1]=(abc-yyxz2)*deta/::fmax(d[1],1e-20);
  di[2]=(abc-zzxy2)*deta/::fmax(d[2],1e-20);
  di[3]=(d[4]*d[5]-d[3]*d[2])*deta;
  di[4]=(d[5]*d[3]-d[4]*d[1])*deta;
  di[5]=(d[3]*d[4]-d[5]*d[0])*deta;
  for(double value:di) if(!tl::math::Finite(value)) return false;
  return true;
}
} // namespace tl::fea::qeph::detail
