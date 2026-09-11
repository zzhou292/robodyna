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
bool Range(const std::array<double,ValueCount>& a, const std::array<double,ValueCount>& b,
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
bool Agree(const std::array<double,ValueCount>& a, const std::array<double,ValueCount>& b,
           double relative) {
  if (!Range(a,b,0,9,relative) || !Range(a,b,9,24,relative) ||
      !Range(a,b,33,9,relative) || !Range(a,b,42,12,relative)) return false;
  for (unsigned p = 0; p < 8; ++p) {
    const unsigned i = 54+10*p;
    if (!Range(a,b,i,9,relative) || !Range(a,b,i+9,1,relative)) return false;
  }
  return Range(a,b,134,2,relative) && Range(a,b,136,1,relative) &&
         Range(a,b,137,1,relative) && Range(a,b,138,9,relative) &&
         Range(a,b,147,1,relative);
}
namespace {
bool UnitGeometry(const std::array<double,ValueCount>& a, const std::array<double,ValueCount>& b,
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
bool AgreeWorkingUnits(const std::array<double,ValueCount>& a, const std::array<double,ValueCount>& b,
                       double coordinate_conditioning) {
  if (!std::isfinite(coordinate_conditioning) || coordinate_conditioning <= 0) return false;
  const double unit_roundoff = 256*std::numeric_limits<double>::epsilon()*
                              std::max(1.0,coordinate_conditioning);
  if (!Range(a,b,0,9,unit_roundoff) || !Range(a,b,9,24,unit_roundoff) ||
      !UnitGeometry(a,b,33,9,unit_roundoff) ||
      !UnitGeometry(a,b,42,12,unit_roundoff)) return false;
  for (unsigned p = 0; p < 8; ++p) {
    const unsigned i = 54+10*p;
    if (!UnitGeometry(a,b,i,9,unit_roundoff) ||
        !Range(a,b,i+9,1,unit_roundoff)) return false;
  }
  return Range(a,b,134,2,unit_roundoff) && Range(a,b,136,1,unit_roundoff) &&
         Range(a,b,137,1,unit_roundoff) && Range(a,b,138,9,unit_roundoff) &&
         Range(a,b,147,1,unit_roundoff);
}
std::array<double,ValueCount> NativeWorkingToSI(const std::array<double,ValueCount>& original) {
  auto values = original;
  for (unsigned i = 9; i < 54; ++i) values[i] *= .001;
  for (unsigned p = 0; p < 8; ++p) {
    const unsigned i = 54+10*p;
    for (unsigned j = i; j < i+9; ++j) values[j] *= .001;
    values[i+9] *= 1e-9;
  }
  values[134] *= 1e-9;
  values[135] *= 1e-9;
  values[136] /= 1e-6;
  values[137] *= .001;
  for (unsigned i = 138; i < 147; ++i) values[i] *= 1000;
  values[147] *= 1e12;
  return values;
}
}  // namespace solid18_test
