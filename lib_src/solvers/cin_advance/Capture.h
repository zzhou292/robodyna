// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Input.h"

#if defined(__CUDACC__)
#define TL_CIN_CAPTURE_HD __host__ __device__
#else
#define TL_CIN_CAPTURE_HD
#endif

namespace tl::fea::cin_advance::capture {
inline constexpr unsigned Threads = 128;
inline constexpr unsigned MaximumBlocks = 256;
inline unsigned Blocks(std::uint32_t nodes) noexcept {
  const auto blocks = nodes/Threads+(nodes%Threads != 0);
  return blocks < MaximumBlocks ? blocks : MaximumBlocks;
}

// Called only after the complete ordered suffix succeeds. Each row owns its
// six output values; rigid members retain the capture written by that suffix.
TL_CIN_CAPTURE_HD inline void CopyNode(const Input& input, std::uint32_t node) {
  if (input.groups.member_nodes && input.groups.member_nodes[node]) return;
  const auto n = input.model.node_count;
  const auto* acceleration = input.work+3*n;
  const auto* angular_acceleration = input.work+6*n;
  for (unsigned a = 0; a < 3; ++a) {
    input.capture.node[3*node+a] = acceleration[3*node+a];
    input.capture.node_rotation[3*node+a] = angular_acceleration[3*node+a];
  }
}

cudaError_t Launch(const Input&, cudaStream_t);
} // namespace tl::fea::cin_advance::capture
#undef TL_CIN_CAPTURE_HD
