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

using tl::math::UnitQuaternion;
using tl::math::IncrementWorldRotation;

TL_NODAL_ROTATION_HD inline tl::math::Quaternion ReadQuaternion(const double* q) {
  return {q[0], q[1], q[2], q[3]};
}

}  // namespace tl::fea::nodal_detail

#undef TL_NODAL_ROTATION_HD
