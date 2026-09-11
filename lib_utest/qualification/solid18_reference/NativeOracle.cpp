// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include <limits>

extern "C" void solid18_reference_native(const double*, const double*, double*, int*, int*);
namespace solid18_test {
NativePacket Native(const s::ReferenceInput& input) {
  double positions[24];
  for (unsigned n = 0; n < 8; ++n) {
    positions[3*n] = input.position_m[n].x;
    positions[3*n+1] = input.position_m[n].y;
    positions[3*n+2] = input.position_m[n].z;
  }
  NativePacket result;
  solid18_reference_native(positions, &input.density_kg_m3, result.values.data(),
                           result.source_slot.data(), &result.status);
  return result;
}
namespace {
bool Range(const std::array<double,348>& a, const std::array<double,348>& b,
           unsigned first, unsigned count, double relative) {
  double scale = 0;
  for (unsigned i = first; i < first+count; ++i) {
    if (!std::isfinite(a[i]) || !std::isfinite(b[i])) return false;
    scale = std::max({scale, std::abs(a[i]), std::abs(b[i])});
  }
  const double roundoff = 64*std::numeric_limits<double>::epsilon()*scale;
  for (unsigned i = first; i < first+count; ++i) {
    if (std::abs(a[i]-b[i]) > relative*std::max(std::abs(a[i]),std::abs(b[i]))+roundoff)
      return false;
  }
  return true;
}
}
bool Agree(const std::array<double,348>& a, const std::array<double,348>& b,
           double relative) {
  if (!Range(a,b,0,9,relative) || !Range(a,b,9,24,relative)) return false;
  for (unsigned p = 0; p < 8; ++p) {
    const unsigned i = 33+33*p;
    if (!Range(a,b,i,8,relative) || !Range(a,b,i+8,24,relative) ||
        !Range(a,b,i+32,1,relative)) return false;
  }
  return Range(a,b,297,9,relative) && Range(a,b,306,24,relative) &&
         Range(a,b,330,1,relative) && Range(a,b,331,8,relative) &&
         Range(a,b,339,8,relative) && Range(a,b,347,1,relative);
}
namespace {
bool UnitDerivatives(const std::array<double,348>& a, const std::array<double,348>& b,
                     unsigned first, unsigned count, double unit_roundoff) {
  double scale = 0;
  for (unsigned i = first; i < first+count; ++i) {
    if (!std::isfinite(a[i]) || !std::isfinite(b[i])) return false;
    scale = std::max({scale,std::abs(a[i]),std::abs(b[i])});
  }
  const double bound = unit_roundoff*scale+64*std::numeric_limits<double>::epsilon()*scale;
  for (unsigned i = first; i < first+count; ++i) {
    if (std::abs(a[i]-b[i]) > bound) return false;
  }
  return true;
}
}
bool AgreeWorkingUnits(const std::array<double,348>& a, const std::array<double,348>& b,
                       double coordinate_conditioning) {
  if (!std::isfinite(coordinate_conditioning) || coordinate_conditioning <= 0) return false;
  const double unit_roundoff = 256*std::numeric_limits<double>::epsilon()*
                              std::max(1.0,coordinate_conditioning);
  // Non-derivative channels keep the previous component-relative contract.
  if (!Range(a,b,0,9,unit_roundoff) || !Range(a,b,9,24,unit_roundoff)) return false;
  for (unsigned p = 0; p < 8; ++p) {
    const unsigned i = 33+33*p;
    for (unsigned n = 0; n < 8; ++n) {
      if (!std::isfinite(a[i+n]) || a[i+n] != b[i+n]) return false;
    }
    if (!UnitDerivatives(a,b,i+8,24,unit_roundoff) ||
        !Range(a,b,i+32,1,unit_roundoff)) return false;
  }
  return Range(a,b,297,9,unit_roundoff) && UnitDerivatives(a,b,306,24,unit_roundoff) &&
         Range(a,b,330,1,unit_roundoff) && Range(a,b,331,8,unit_roundoff) &&
         Range(a,b,339,8,unit_roundoff) && Range(a,b,347,1,unit_roundoff);
}
std::array<double,348> NativeWorkingToSI(const std::array<double,348>& original) {
  auto values = original;
  for (unsigned i = 9; i < 33; ++i) values[i] *= .001;
  for (unsigned p = 0; p < 8; ++p) {
    const unsigned i = 33+33*p;
    for (unsigned j = i+8; j < i+32; ++j) values[j] /= .001;
    values[i+32] *= 1e-9;
  }
  for (unsigned i = 297; i < 306; ++i) values[i] *= 1e-9;
  for (unsigned i = 306; i < 330; ++i) values[i] /= .001;
  values[330] *= .001;
  for (unsigned i = 331; i < 339; ++i) values[i] *= 1000;
  for (unsigned i = 339; i < 347; ++i) values[i] *= 1e-9;
  values[347] *= 1000;
  return values;
}
}  // namespace solid18_test
