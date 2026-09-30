#pragma once
#include "../FENodalStateStorage.h"
#include "Rows.cuh"

// Private seal scheduling. Only the independent finite/rotation checks run in
// parallel. Revisit the earliest failing node in axis order. Optional row
// scratch also parallelizes validation/maxima before the unchanged scalar
// stability formula. No force or accepted state is changed.
namespace tl::fea::nodal_seal {
namespace sc = tlfea::contact;
using nodal_detail::Control;
static __global__ void Begin(Control* c, std::uint64_t epoch, std::uint64_t attempt) {
  if (c->status != NodalStatus::Ok) return;
  if (c->assembly.base_epoch != epoch || c->assembly.attempt != attempt ||
      c->rows.base_epoch != epoch || c->rows.attempt != attempt) {
    c->status = NodalStatus::StaleTrial; return;
  }
  if (c->assembly.status != sc::Status::kOk) {
    c->status = NodalStatus::ContributorFailure; c->node = c->assembly.node; return;
  }
  c->node = UINT32_MAX;
}
static __global__ void FindFailure(Control* c, const double* scratch, std::uint32_t n, bool rotations) {
  if (c->status != NodalStatus::Ok) return;
  for (std::uint32_t i=blockIdx.x*blockDim.x+threadIdx.x; i<n; i+=blockDim.x*gridDim.x) {
    for (unsigned axis=0; axis<6; ++axis) {
      const double value=scratch[axis*n+i];
      if (!sc::IsFinite(value) || (!rotations && axis>=3 && value!=0)) {
        atomicMin(&c->node,i); break;
      }
    }
  }
}
static __global__ void Finish(Control* c, const double* scratch, std::uint32_t n,
                        double safety, double minimum_dt, double h, bool rotations, RowScratch rows) {
  if (c->status != NodalStatus::Ok) return;
  if (c->node != UINT32_MAX) {
    for (unsigned axis=0; axis<6; ++axis) {
      const double value=scratch[axis*n+c->node];
      if (!sc::IsFinite(value)) { c->status=NodalStatus::InvalidOutput; return; }
      if (!rotations && axis>=3 && value!=0) { c->status=NodalStatus::UnsupportedRotation; return; }
    }
  }
  auto status = sc::Status::kOk;
  if (rows.summaries && CanReduceRows(c->rows, scratch, n, safety, minimum_dt, h)) {
    auto summary = EmptyRows();
    for (std::uint32_t block = 0; block < rows.blocks; ++block)
      CombineRows(summary, rows.summaries[block]);
    status = FinalizeReducedRows(&c->rows, safety, minimum_dt, h, summary, &c->limit);
  } else {
    status = stability::FinalizeRows(&c->rows, safety, minimum_dt, h, &c->limit);
  }
  if (status != sc::Status::kOk) {
    c->status = status == sc::Status::kOutOfRange ? NodalStatus::StepTooLarge : NodalStatus::InvalidOutput;
    return;
  }
  if (!stability::IsCurrentLimit(c->rows, c->limit)) { c->status = NodalStatus::StaleTrial; return; }
  if (c->limit.dt < h) {
    c->status = NodalStatus::StepTooLarge;
    c->node = c->limit.stiffness_bound > 0 ? c->limit.stiffness_node : c->limit.damping_node;
  }
}
static inline void Launch(Control* c, const double* scratch, std::uint32_t n,
                   std::uint64_t epoch, std::uint64_t attempt,
                   double safety, double minimum_dt, double h, bool rotations, cudaStream_t stream,
                   RowScratch rows = {}) {
  Begin<<<1,1,0,stream>>>(c,epoch,attempt);
  if (cudaPeekAtLastError()!=cudaSuccess) return;
  const auto requested=(n+255u)/256u;
  FindFailure<<<requested<256u?requested:256u,256,0,stream>>>(c,scratch,n,rotations);
  if (cudaPeekAtLastError()!=cudaSuccess) return;
  // A missing/unexpected private tail selects the exact legacy path.
  if (!rows.summaries || rows.blocks != RowBlocks(n) ||
      rows.summaries != ControlTail(c,n).summaries) rows = {};
  if (rows.summaries) {
    ReduceRows<<<rows.blocks,RowThreads,0,stream>>>(c,scratch,n,safety,minimum_dt,h,rows.summaries);
    if (cudaPeekAtLastError()!=cudaSuccess) return;
  }
  Finish<<<1,1,0,stream>>>(c,scratch,n,safety,minimum_dt,h,rotations,rows);
}
} // namespace tl::fea::nodal_seal
