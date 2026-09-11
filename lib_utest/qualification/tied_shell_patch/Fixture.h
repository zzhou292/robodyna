#pragma once
#include "lib_src/constraints/tied_shell/TiedPatchForce.h"
#include "lib_src/constraints/tied_shell/TiedPatchMotion.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

namespace tied_patch_test {
namespace tie = tl::constraints::tied_shell;
namespace math = tl::math::fixed3;
using tie::Vec3;

inline tie::PatchInput Geometry(unsigned shape) {
  tie::PatchInput input{{{-.021,-.013,.003}, {.024,-.015,-.002},
                        {.018,.017,.004}, {-.016,.012,-.001}}, {.011,-.007,.006}};
  if (shape == 1) input.master_position[3] = input.master_position[2];
  if (shape == 2) {
    for (auto& x : input.master_position) x.z = 0;
  }
  if (shape == 3) {
    // Fixed exact orthogonal permutation plus original-vehicle sized translation.
    for (auto& x : input.master_position) x = {x.z+3.2, x.x+.7, x.y+1.1};
    const auto x = input.secondary_position;
    input.secondary_position = {x.z+3.2, x.x+.7, x.y+1.1};
  }
  return input;
}
inline tie::MasterMotion Motion(unsigned step, bool repeated_node = false) {
  tie::MasterMotion motion;
  for (unsigned i = 0; i < 4; ++i) {
    const double a = i + 1, b = step + 1;
    motion.velocity[i] = {.11*a*b, -.031*a*a+.04*b, .028*a-.007*b*b};
    motion.acceleration[i] = {1.3*a-b, -.7*a*b, .13*a*a+.3*b};
  }
  if (repeated_node) {
    motion.velocity[3] = motion.velocity[2];
    motion.acceleration[3] = motion.acceleration[2];
  }
  return motion;
}
inline tie::SecondaryLoad Load(unsigned step) {
  return {{110.+13.*step, -83.+.7*step, 47.-2.*step}, {.3+.1*step, -.19, .083-.02*step}};
}
inline void Append(std::vector<double>& values, Vec3 v) {
  values.insert(values.end(), {v.x,v.y,v.z});
}
inline std::vector<double> Values(const tie::MasterLoads& load, const tie::SecondaryMotion& motion) {
  std::vector<double> values;
  for (const auto& f : load.force) Append(values, f);
  Append(values,motion.velocity); Append(values,motion.angular_velocity);
  Append(values,motion.acceleration); Append(values,motion.angular_acceleration);
  return values;
}
inline void Near(double actual, double expected) {
  ASSERT_TRUE(std::isfinite(actual)); ASSERT_TRUE(std::isfinite(expected));
  EXPECT_LE(std::abs(actual-expected), 2e-11*std::max(1.,std::abs(expected)));
}
inline void Near(Vec3 a, Vec3 b) { Near(a.x,b.x); Near(a.y,b.y); Near(a.z,b.z); }
template<class T> auto Bytes(const T& value) {
  std::array<unsigned char,sizeof(T)> bytes;
  std::memcpy(bytes.data(),&value,sizeof(T));
  return bytes;
}
inline auto PatchBits(const tie::Patch& patch) {
  const auto& p = patch.values();
  std::vector<double> values;
  Append(values,p.center);
  for (const auto& x : p.master_offset) Append(values,x);
  Append(values,p.secondary_offset);
  values.insert(values.end(),p.cofactor,p.cofactor+7);
  std::array<std::uint64_t,25> bits;
  std::memcpy(bits.data(),values.data(),sizeof(bits));
  return bits;
}
} // namespace tied_patch_test
