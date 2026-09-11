// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NativeOracle.h"
#include "Values.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <iomanip>

namespace solid6z_force_test {
inline ::testing::AssertionResult Group(const char* name,const double* a,const double* b,
    unsigned begin,unsigned end,double roundoff_scale = 0) {
  double scale = roundoff_scale;
  for (unsigned i = begin; i < end; ++i) scale = std::max({scale,std::abs(a[i]),std::abs(b[i])});
  for (unsigned i = begin; i < end; ++i) {
    if (!std::isfinite(a[i]) || !std::isfinite(b[i]) || std::abs(a[i]-b[i]) > 3e-10*scale) {
      return ::testing::AssertionFailure() << name << '[' << i << "] actual=" << std::setprecision(17)
          << a[i] << " native=" << b[i] << " scale=" << scale;
    }
  }
  return ::testing::AssertionSuccess();
}
inline ::testing::AssertionResult Agree(const Values& a,const NativeResult& b) {
  if (b.status != 0) return ::testing::AssertionFailure() << "native status " << b.status;
  const unsigned geometry[]{0,9,27,45,63,72,81,90,96,97,98};
  for (unsigned k = 0; k+1 < sizeof(geometry)/sizeof(*geometry); ++k) {
    const auto check = Group("geometry",a.geometry,b.geometry.data(),geometry[k],geometry[k+1]);
    if (!check) return check;
  }
  const unsigned material[]{0,6,7,8,9,15,17,18,19,20,21,22,28,29,30,31,32,33};
  for (unsigned k = 0; k+1 < sizeof(material)/sizeof(*material); ++k) {
    const auto check = Group("material",a.material,b.material.data(),material[k],material[k+1]);
    if (!check) return check;
  }
  const unsigned history[]{0,6,7,8,9,21};
  for (unsigned k = 0; k+1 < sizeof(history)/sizeof(*history); ++k) {
    const auto check = Group("history",a.history,b.history.data(),history[k],history[k+1]);
    if (!check) return check;
  }
  for (unsigned k = 0; k < 3; ++k) {
    const auto check = Group("force",a.forces,b.forces.data(),18*k,18*(k+1));
    if (!check) return check;
  }
  const unsigned stabilization[]{0,12,24,25,26,27,28};
  for (unsigned k = 0; k+1 < sizeof(stabilization)/sizeof(*stabilization); ++k) {
    const auto check = Group("stabilization",a.stabilization,b.stabilization.data(),stabilization[k],stabilization[k+1]);
    if (!check) return check;
  }
  if (a.material[17] != 1 || b.material[17] != 1)
    return ::testing::AssertionFailure() << "active profile changed";
  return ::testing::AssertionSuccess();
}
} // namespace solid6z_force_test
