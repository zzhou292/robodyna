#pragma once
#include "../NodalRotation.h"
#include <cstdint>

namespace tl::fea::nodal_motion {
struct Summary {
  std::uint64_t nonfinite, quaternion, motion;
  double position, velocity, orientation, spin;
};
__host__ __device__ inline Summary Empty() {
  return {UINT64_MAX, UINT64_MAX, UINT64_MAX, 0, 0, 0, 0};
}
__host__ __device__ inline void Minimum(std::uint64_t& a, std::uint64_t b) {
  if (b < a) a = b;
}
__host__ __device__ inline void Maximum(double& a, double b) {
  if (a < b) a = b;
}
__host__ __device__ inline void Merge(Summary& a, const Summary& b) {
  Minimum(a.nonfinite, b.nonfinite);
  Minimum(a.quaternion, b.quaternion);
  Minimum(a.motion, b.motion);
  Maximum(a.position, b.position);
  Maximum(a.velocity, b.velocity);
  Maximum(a.orientation, b.orientation);
  Maximum(a.spin, b.spin);
}
__host__ __device__ inline void Difference(Summary& out, double actual,
    double expected, double& maximum, std::uint64_t ordinal) {
  if (!tl::math::Finite(actual) || !tl::math::Finite(expected)) {
    Minimum(out.motion, ordinal);
    return;
  }
  const double difference = ::fabs(actual - expected);
  if (!tl::math::Finite(difference)) Minimum(out.motion, ordinal);
  else Maximum(maximum, difference);
}
} // namespace tl::fea::nodal_motion
