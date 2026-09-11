// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Solid18Types.h"

namespace tl::fea::solid18::detail {
struct Basis {
  double h[8]{}, r[8]{}, s[8]{}, t[8]{};
};

TL_SOLID18_HD inline Basis StartupBasis(unsigned point) noexcept {
  // BASISF's default-REAL DATA literal, promoted to the selected binary64 build.
  constexpr double gauss = static_cast<double>(.5773502691896f);
  constexpr int sign[8][3] = {
    {-1,-1,-1},{1,-1,-1},{1,1,-1},{-1,1,-1},
    {-1,-1,1},{1,-1,1},{1,1,1},{-1,1,1}
  };
  const double r = sign[point][0]*gauss;
  const double s = sign[point][1]*gauss;
  const double t = sign[point][2]*gauss;
  const double rp = 1+r, sp = 1+s, tp = 1+t;
  const double rm = 1-r, sm = 1-s, tm = 1-t;
  Basis b;
  b.h[0] = .125*rm*sm*tm;
  b.h[1] = .125*rp*sm*tm;
  b.h[2] = .125*rp*sp*tm;
  b.h[3] = .125*rm*sp*tm;
  b.h[4] = .125*rm*sm*tp;
  b.h[5] = .125*rp*sm*tp;
  b.h[6] = .125*rp*sp*tp;
  b.h[7] = .125*rm*sp*tp;
  b.r[0] = -.125*sm*tm;
  b.r[1] = -b.r[0];
  b.r[2] = .125*sp*tm;
  b.r[3] = -b.r[2];
  b.r[4] = -.125*sm*tp;
  b.r[5] = -b.r[4];
  b.r[6] = .125*sp*tp;
  b.r[7] = -b.r[6];
  b.s[0] = -.125*rm*tm;
  b.s[1] = -.125*rp*tm;
  b.s[2] = -b.s[1];
  b.s[3] = -b.s[0];
  b.s[4] = -.125*rm*tp;
  b.s[5] = -.125*rp*tp;
  b.s[6] = -b.s[5];
  b.s[7] = -b.s[4];
  b.t[0] = -.125*rm*sm;
  b.t[1] = -.125*rp*sm;
  b.t[2] = -.125*rp*sp;
  b.t[3] = -.125*rm*sp;
  b.t[4] = -b.t[0];
  b.t[5] = -b.t[1];
  b.t[6] = -b.t[2];
  b.t[7] = -b.t[3];
  return b;
}
}  // namespace tl::fea::solid18::detail
