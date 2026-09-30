// SPDX-License-Identifier: AGPL-3.0-or-later
#include "CrossingExecutor.h"
#include "../SelfContactTransactionTypes.h"
#include "../represented_interval_crossing/DeviceBatch.h"

namespace tlfea::contact::self_contact_transaction {
RepresentedIntervalGpuReport CrossingExecutor::Initialize(const SelfContactTransactionConfig& config,
    const SelfContactTransactionLimits& limits, cudaStream_t stream) noexcept {
  RepresentedIntervalGpuReport result;
  if (initialized_) {
    result.native.status = RepresentedIntervalStatus::AlreadyInitialized;
    result.native.message = "Crossing executor is already initialized";
    return result;
  }
  if (config.enable_cuda_native_crossing)
    result = gpu_.Initialize(CrossingGpuLimits(config, limits), stream);
  else
    result.native = cpu_.Initialize(limits.crossing);
  if (result.native.status != RepresentedIntervalStatus::Ok) return result;
  use_gpu_ = config.enable_cuda_native_crossing;
  stream_ = stream;
  initialized_ = true;
  return result;
}
CrossingExecutionReport CrossingExecutor::Certify(const RepresentedTrianglePath* paths,
    std::size_t path_count, const RepresentedTrianglePair* pairs, std::size_t pair_count,
    std::size_t batch_capacity, RepresentedIntervalResult* output, std::size_t output_capacity) noexcept {
  CrossingExecutionReport result;
  if (!initialized_) {
    result.native.status = RepresentedIntervalStatus::NotInitialized;
    result.native.message = "Crossing executor is not initialized";
    return result;
  }
  if (use_gpu_) {
    const auto gpu = represented_interval_crossing::DeviceBatchAccess::Certify(gpu_,
        paths, path_count, pairs, pair_count, batch_capacity, output, output_capacity, stream_);
    result.native = gpu.native;
    result.device = gpu.device;
  } else {
    result.native = CertifyCrossingBatches(cpu_, paths, path_count, pairs, pair_count,
        batch_capacity, output, output_capacity);
  }
  return result;
}
RepresentedIntervalResultView CrossingExecutor::results() const noexcept {
  if (!initialized_) return {};
  return use_gpu_ ? gpu_.results() : cpu_.results();
}
}  // namespace tlfea::contact::self_contact_transaction
