// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Copies.h"
#include "lib_src/collision/self_contact_filters/Types.h"
#include <atomic>
#include <algorithm>
#include <cuda_runtime_api.h>

namespace {
// This dedicated benchmark invokes one externally serialized adapter. The
// wrapper only observes successful calls and always forwards their real result.
std::atomic<bool> observing{false};
facet_filter_benchmark::Copies counters;
}
extern "C" cudaError_t __real_cudaMemcpyAsync(void*, const void*, std::size_t,
                                            cudaMemcpyKind, cudaStream_t);
extern "C" cudaError_t __wrap_cudaMemcpyAsync(void* output, const void* input,
    std::size_t bytes, cudaMemcpyKind kind, cudaStream_t stream) {
  const auto result = __real_cudaMemcpyAsync(output, input, bytes, kind, stream);
  if (result == cudaSuccess && observing.load(std::memory_order_relaxed)) {
    if (kind == cudaMemcpyHostToDevice) {
      using Pair = tlfea::contact::FixedTrianglePair;
      ++counters.host_to_device_calls; counters.host_to_device_bytes += bytes;
      if (!bytes || bytes % sizeof(Pair)) counters.unexpected_copy_shape = true;
      const auto pairs = bytes / sizeof(Pair);
      counters.minimum_query_pairs = counters.minimum_query_pairs
          ? std::min(counters.minimum_query_pairs, pairs) : pairs;
      counters.maximum_query_pairs = std::max(counters.maximum_query_pairs, pairs);
    } else if (kind == cudaMemcpyDeviceToHost) {
      ++counters.device_to_host_calls; counters.device_to_host_bytes += bytes;
      if (!bytes || bytes % sizeof(tlfea::contact::self_contact_filters::PairResult))
        counters.unexpected_copy_shape = true;
    } else counters.unexpected_copy_shape = true;
  }
  return result;
}
namespace facet_filter_benchmark {
void BeginCopyObservation() noexcept { counters = {}; observing.store(true, std::memory_order_release); }
Copies EndCopyObservation() noexcept {
  observing.store(false, std::memory_order_release);
  return counters;
}
}  // namespace facet_filter_benchmark
