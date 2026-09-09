// Adapted from Project Chrono, Copyright (c) 2014 projectchrono.org.
// BSD-3-Clause terms are reproduced in the included math/Quaternion.h.
// Sources: core/ChQuaternion.h SetFromRotVec and fea/ChNodeFEAxyzrot.cpp
// NodeIntStateIncrement. Source SHA256, respectively:
// b57b207950dc4d7ddc81c1d952bbcbb7d0db1b318f2919d51f87606acc06a9b7
// 000f33dcf16ead8df210036d0a345c70b7031e3c23c1423f3e6dc8615575a577
// Changes: fixed-size shared TL arithmetic, world rather than local spin,
// explicit finite/unit admission and bounded candidate roundoff projection.
#pragma once

#include "../math/Quaternion.h"
#include <cmath>

#if defined(__CUDACC__)
#define TL_NODAL_ROTATION_HD __host__ __device__
#else
#define TL_NODAL_ROTATION_HD
#endif

namespace tl::fea::nodal_detail {

// One shared startup/step/readback unit check. Invalid input is never repaired.
TL_NODAL_ROTATION_HD inline bool UnitQuaternion(tl::math::Quaternion q) {
  return tl::math::Finite(q) && ::fabs(tl::math::Dot(q, q) - 1) <= 1e-12;
}

TL_NODAL_ROTATION_HD inline tl::math::Quaternion ReadQuaternion(const double* q) {
  return {q[0], q[1], q[2], q[3]};
}

// Chrono ChQuaternion::SetFromRotVec, including its theta^2 <= 1e-30 branch,
// followed by the equivalent WORLD increment from ChNodeFEAxyzrot.cpp:
// q_new = dq_world * q_old (Chrono's node increment stores local omega).
// Input/output are copied values. A candidate already within unit roundoff
// tolerance is projected back to unit length; invalid input/output is rejected.
// Increment-angle admission belongs to the step operation.
TL_NODAL_ROTATION_HD inline bool IncrementWorldRotation(
    tl::math::Quaternion initial, const double increment[3], tl::math::Quaternion& output) {
  if (!UnitQuaternion(initial)) return false;
  const double square = increment[0] * increment[0] + increment[1] * increment[1] + increment[2] * increment[2];
  if (!tl::math::Finite(square) || square < 0) return false;
  double scalar = 1, factor = .5;
  if (square > 1e-30) {
    const double angle = ::sqrt(square);
    scalar = ::cos(.5 * angle);
    factor = ::sin(.5 * angle) / angle;
  }
  const tl::math::Quaternion delta{scalar, factor * increment[0], factor * increment[1], factor * increment[2]};
  const auto candidate = tl::math::Product(delta, initial);
  if (!UnitQuaternion(candidate)) return false;
  const auto normalized = tl::math::Scale(candidate, 1 / ::sqrt(tl::math::Dot(candidate, candidate)));
  if (!UnitQuaternion(normalized)) return false;
  output = normalized;
  return true;
}

}  // namespace tl::fea::nodal_detail

#undef TL_NODAL_ROTATION_HD
