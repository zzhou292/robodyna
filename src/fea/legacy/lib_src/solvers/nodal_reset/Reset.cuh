#pragma once
#include "../FENodalStateStorage.h"

namespace tl::fea::nodal_reset {
inline constexpr unsigned Threads = 256;

// Private owner kernel: launch exactly one block. The actual owner supplies
// disjoint, node_count-sized stiffness/damping ranges in its admitted scratch.
// No caller may observe or contribute to rows until this stream completes reset.
static __global__ void ResetTrial(
    nodal_detail::Control* c, std::uint64_t epoch, std::uint64_t attempt) {
  __shared__ tlfea::contact::Status reset_status;
  if (threadIdx.x == 0) {
    c->assembly = {};
    c->assembly.base_epoch = epoch; c->assembly.attempt = attempt;
    c->limit = {}; c->node = UINT32_MAX; c->status = NodalStatus::Ok;
    c->structural_limiter = {};
    reset_status = stability::detail::BeginResetRows(&c->rows, epoch, attempt);
    if (reset_status != tlfea::contact::Status::kOk)
      c->status = NodalStatus::InvalidOutput;
  }
  __syncthreads();
  // Every lane sees the same result and takes the same rejected return.
  if (reset_status != tlfea::contact::Status::kOk) return;
  for (std::size_t node = threadIdx.x; node < c->rows.node_count; node += blockDim.x) {
    c->rows.stiffness[node] = 0;
    c->rows.damping[node] = 0;
  }
  __syncthreads();
  if (threadIdx.x == 0)
    stability::detail::CompleteResetRows(&c->rows, epoch, attempt);
}
} // namespace tl::fea::nodal_reset
