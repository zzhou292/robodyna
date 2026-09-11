// SPDX-License-Identifier: AGPL-3.0-or-later
// S8EPRST_INI/S8EDERIG3: OpenRadioss, Copyright (C) 2026 Siemens.
#pragma once
#include "Solid18ForceTypes.h"

namespace tl::fea::solid18::detail {
TL_SOLID18_HD inline void NaturalDerivatives(unsigned ip, double (&p)[3][8]) noexcept {
  constexpr double pg = .577350269189625;
  const double pg2 = pg*pg;
  const double r = (ip&1) ? pg : -pg;
  const double s = (ip&2) ? pg : -pg;
  const double t = (ip&4) ? pg : -pg;
  const double rs = ((ip&1) != 0) == ((ip&2) != 0) ? pg2 : -pg2;
  const double rt = ((ip&1) != 0) == ((ip&4) != 0) ? pg2 : -pg2;
  const double st = ((ip&2) != 0) == ((ip&4) != 0) ? pg2 : -pg2;
  p[0][0] = -(1-s-t+st);
  p[0][1] = -p[0][0];
  p[0][2] = 1+s-t-st;
  p[0][3] = -p[0][2];
  p[0][4] = -(1-s+t-st);
  p[0][5] = -p[0][4];
  p[0][6] = 1+s+t+st;
  p[0][7] = -p[0][6];
  p[1][0] = -(1-r-t+rt);
  p[1][1] = -(1+r-t-rt);
  p[1][2] = -p[1][1];
  p[1][3] = -p[1][0];
  p[1][4] = -(1-r+t-rt);
  p[1][5] = -(1+r+t+rt);
  p[1][6] = -p[1][5];
  p[1][7] = -p[1][4];
  p[2][0] = -(1-r-s+rs);
  p[2][1] = -(1+r-s-rs);
  p[2][2] = -(1+r+s+rs);
  p[2][3] = -(1-r+s-rs);
  p[2][4] = -p[2][0];
  p[2][5] = -p[2][1];
  p[2][6] = -p[2][2];
  p[2][7] = -p[2][3];
}

TL_SOLID18_HD inline void RegularDerivatives(unsigned ip, const Matrix3& inverse,
                                            double (&out)[3][8]) noexcept {
  double p[3][8];
  NaturalDerivatives(ip, p);
  for (unsigned axis = 0; axis < 3; ++axis) {
    const double a = inverse.v[3*axis];
    const double b = inverse.v[3*axis+1];
    const double c = inverse.v[3*axis+2];
    const double ar1 = a*p[0][0], ar3 = a*p[0][2];
    const double ar5 = a*p[0][4], ar7 = a*p[0][6];
    const double bs1 = b*p[1][0], bs2 = b*p[1][1];
    const double bs5 = b*p[1][4], bs6 = b*p[1][5];
    const double ct1 = c*p[2][0], ct2 = c*p[2][1];
    const double ct3 = c*p[2][2], ct4 = c*p[2][3];
    out[axis][0] = ar1+bs1+ct1;
    out[axis][1] = -ar1+bs2+ct2;
    out[axis][2] = ar3-bs2+ct3;
    out[axis][3] = -ar3-bs1+ct4;
    out[axis][4] = ar5+bs5-ct1;
    out[axis][5] = -ar5+bs6-ct2;
    out[axis][6] = ar7-bs6-ct3;
    out[axis][7] = -ar7-bs5-ct4;
  }
}
}  // namespace tl::fea::solid18::detail
