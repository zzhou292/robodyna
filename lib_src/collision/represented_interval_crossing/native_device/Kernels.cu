// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Workspace.h"
#include "KernelTypes.h"
#include "../NativeStorageDomain.h"
#include <new>

namespace tlfea::contact::represented_interval_crossing::native_device {
namespace {
__global__ __launch_bounds__(ThreadsPerBlock, 1)
void Certify(const RepresentedTrianglePath* paths, std::size_t path_count,
    const DeviceJob* jobs, std::size_t job_count, RepresentedIntervalLimits limits,
    unsigned worker_count, ExactScratch* scratch, native::Cell* dfs,
    std::size_t dfs_capacity, DeviceResult* results) {
  const unsigned worker = blockIdx.x * blockDim.x + threadIdx.x;
  if (worker >= worker_count) return;
  // Scratch and DFS live in the explicitly forecast retained arena. Per-pair
  // arithmetic context is lexical; no device global state or float atomics.
  auto* local_scratch = ::new (static_cast<void*>(scratch + worker)) ExactScratch{};
  auto* local_dfs = dfs + worker * dfs_capacity;
  for (std::size_t i = 0; i < dfs_capacity; ++i)
    ::new (static_cast<void*>(local_dfs + i)) native::Cell{};
  for (std::size_t i = worker; i < job_count; i += worker_count) {
    auto* output = ::new (static_cast<void*>(results + i)) DeviceResult{};
    const auto& pair = jobs[i].pair;
    if (pair.first >= path_count || pair.second >= path_count || pair.first == pair.second) {
      output->execution = PairExecution::InvalidIndex;
      continue;
    }
    const auto& a = paths[pair.first];
    const auto& b = paths[pair.second];
    if (!native::Same(a.key, pair.key.paths[0]) || !native::Same(b.key, pair.key.paths[1]) ||
        native::Compare(a.key, b.key) >= 0) {
      output->execution = PairExecution::IdentityMismatch;
      continue;
    }
    // Recompute the real coordinate/depth proof. No host eligibility bit is
    // authority on the device. Domain disagreement is never a CPU retry.
    if (!NativeStorageDomain::FromPaths(a, b, limits.max_depth).eligible()) {
      output->execution = PairExecution::DomainMismatch;
      continue;
    }
    native::ArithmeticContext context;
    Kernel kernel(context);
    const auto result = kernel.CertifyPair<native::NormalReuse::Memoize,
        native::SeparationProof::RelativeFaces, native::ExactPathReuse::Optimized,
        native::CommonPointReuse::Optimized>(a, b, limits, pair.key,
            local_dfs, dfs_capacity, local_scratch);
    native::StoreResult(result, &output->value);
    output->execution = PairExecution::Complete;
  }
}
}  // namespace
cudaError_t Launch(void* device, const Layout& layout, std::size_t path_count,
    std::size_t job_count, RepresentedIntervalLimits limits, cudaStream_t stream) noexcept {
  if (!job_count) return cudaSuccess;
  const auto workers = static_cast<unsigned>(job_count < layout.forecast.device_workers
      ? job_count : layout.forecast.device_workers);
  const unsigned blocks = (workers + ThreadsPerBlock - 1) / ThreadsPerBlock;
  Certify<<<blocks, ThreadsPerBlock, 0, stream>>>(
      tl::util::ArenaPointer<RepresentedTrianglePath>(device, layout.paths), path_count,
      tl::util::ArenaPointer<DeviceJob>(device, layout.jobs), job_count, limits, workers,
      tl::util::ArenaPointer<ExactScratch>(device, layout.exact_scratch),
      tl::util::ArenaPointer<native::Cell>(device, layout.dfs), layout.dfs_capacity,
      tl::util::ArenaPointer<DeviceResult>(device, layout.results));
  return cudaPeekAtLastError();
}
}  // namespace tlfea::contact::represented_interval_crossing::native_device
