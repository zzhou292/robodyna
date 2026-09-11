// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/solvers/cin_advance/Input.h"
#if defined(__CUDACC__)
#define TL_CIN_FROZEN_HD __host__ __device__
#else
#define TL_CIN_FROZEN_HD
#endif
namespace tl::fea::cin_capture_test {
TL_CIN_FROZEN_HD inline void CopyFrozen(const cin_advance::Input& input) {
  const auto n = input.model.node_count;
  const auto groups = input.groups;
  const auto capture = input.capture;
  const auto* acceleration = input.work+3*n;
  const auto* angular_acceleration = input.work+6*n;
  if (capture.node) {
    for (std::uint32_t node = 0; node < n; ++node) {
      if (groups.member_nodes && groups.member_nodes[node]) continue;
      for (unsigned a = 0; a < 3; ++a) {
        capture.node[3*node+a] = acceleration[3*node+a];
        capture.node_rotation[3*node+a] = angular_acceleration[3*node+a];
      }
    }
  }
}
} // namespace tl::fea::cin_capture_test
#undef TL_CIN_FROZEN_HD
