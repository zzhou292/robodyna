// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodalRigidGroupStepMath.h"

namespace tl::fea::rigid {
// Private, bounded by owner startup. No identity, clock or allocation. Only the
// Capture=true candidate specialization writes these transient output ranges.
struct AccelerationSink {
  double *node=nullptr,*node_rotation=nullptr,*group=nullptr,*group_rotation=nullptr;
#if defined(__CUDACC__)
  __host__ __device__
#endif
  static void Write(double* out,std::uint32_t index,Vec3 value) {
    out[3*index]=value.x;out[3*index+1]=value.y;out[3*index+2]=value.z;
  }
};
} // namespace tl::fea::rigid
