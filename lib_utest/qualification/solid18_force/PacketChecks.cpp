// SPDX-License-Identifier: AGPL-3.0-or-later
#include "PacketValues.h"

namespace solid18_force_test {
bool Agree(const s::ForceTrial& actual, const NativeResult& expected) {
  if (expected.status != 0) return false;
  const auto a = Values(actual);
  for (unsigned ip = 0; ip < 8; ++ip) {
    SCOPED_TRACE("engine point "+std::to_string(ip));
    const auto* x = a.next.point.data()+20*ip;
    const auto* y = expected.next.point.data()+20*ip;
    if (!Range(x,y,6,"point stress") || !Range(x+6,y+6,6,"point strain")) return false;
    for (unsigned k = 12; k < 20; ++k) {
      if (!Range(x+k,y+k,1,"point history")) return false;
    }
    x = a.observation.data()+38*ip;
    y = expected.observation.data()+38*ip;
    if (!Range(x,y,6,"material stress") || !Range(x+6,y+6,6,"material strain")) return false;
    for (unsigned k = 12; k < 26; ++k) {
      if (!Range(x+k,y+k,1,"material diagnostic")) return false;
    }
    if (!Range(x+26,y+26,6,"rate")) return false;
    if (!Range(x+32,y+32,1,"selective volume increment") ||
        !Range(x+33,y+33,1,"storage volume factor")) return false;
    const double volume_a[] = {x[34],actual.geometry.point[ip].current_volume_m3};
    const double volume_b[] = {y[34],expected.geometry[71+130*ip+129]};
    if (!Range(volume_a,volume_b,2,"volume increment/current volume")) return false;
    for (unsigned k = 35; k < 38; ++k) {
      if (!Range(x+k,y+k,1,"viscosity/timestep/stiffness")) return false;
    }
  }
  if (!Range(a.next.global.data(),expected.next.global.data(),6,"global stress")) return false;
  for (unsigned k = 6; k < 11; ++k) {
    if (!Range(a.next.global.data()+k,expected.next.global.data()+k,1,"global history")) return false;
  }
  if (!Range(a.next.saved.data(),expected.next.saved.data(),21,"saved local positions") ||
      !Range(a.force.data(),expected.force.data(),24,"source force")) return false;
  const auto* x = a.geometry.data();
  const auto* y = expected.geometry.data();
  if (!Range(x,y,9,"frame") || !Range(x+9,y+9,24,"local positions") ||
      !Range(x+33,y+33,24,"local velocity") || !Range(x+57,y+57,12,"center gradients") ||
      !Range(x+69,y+69,1,"center volume") || !Range(x+70,y+70,1,"face scale")) return false;
  for (unsigned ip = 0; ip < 8; ++ip) {
    const unsigned i = 71+130*ip;
    if (!Range(x+i,y+i,24,"regular gradient") || !Range(x+i+24,y+i+24,48,"shear gradient") ||
        !Range(x+i+72,y+i+72,48,"cross gradient") ||
        !Range(x+i+120,y+i+120,9,"inverse Jacobian") ||
        !Range(x+i+129,y+i+129,1,"point volume")) return false;
  }
  if (a.diagnostics[2] != expected.diagnostics[2]) {
    ADD_FAILURE() << "Native selected point differs";
    return false;
  }
  for (unsigned k = 0; k < 7; ++k) {
    if (!Range(a.diagnostics.data()+k,expected.diagnostics.data()+k,1,"force diagnostic")) return false;
  }
  return true;
}
} // namespace solid18_force_test
