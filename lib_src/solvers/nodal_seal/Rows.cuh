#pragma once
#include "RowLayout.h"

namespace tl::fea::nodal_seal {
static __global__ void ReduceRows(const nodal_detail::Control* control, const double* scratch,
    std::uint32_t n, double safety, double minimum_dt, double h, RowSummary* summaries) {
  if (control->status != NodalStatus::Ok || control->node != UINT32_MAX ||
      !CanReduceRows(control->rows, scratch, n, safety, minimum_dt, h)) return;
  auto local = EmptyRows();
  // Widen index arithmetic so the strided last iteration cannot wrap at the
  // admitted count boundary. Reads remain inside the actual owner's two rows.
  for (std::size_t i = blockIdx.x * blockDim.x + threadIdx.x; i < n;
       i += std::size_t(blockDim.x) * gridDim.x) {
    IncludeRow(local, control->rows.stiffness[i], control->rows.damping[i],
               static_cast<std::uint32_t>(i));
  }
  __shared__ RowSummary lanes[RowThreads];
  lanes[threadIdx.x] = local;
  __syncthreads();
  for (unsigned stride = RowThreads / 2; stride; stride /= 2) {
    if (threadIdx.x < stride) CombineRows(lanes[threadIdx.x], lanes[threadIdx.x + stride]);
    __syncthreads();
  }
  if (!threadIdx.x) summaries[blockIdx.x] = lanes[0];
}
} // namespace tl::fea::nodal_seal
