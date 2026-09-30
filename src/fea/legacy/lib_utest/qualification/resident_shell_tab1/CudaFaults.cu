#include "CudaFixture.h"
#include <limits>
namespace resident_tab1_test {
namespace {
ReadFault fault = ReadFault::None;
unsigned copies = 0;
}
void Arm(ReadFault value) { fault = value; copies = 0; }
unsigned Copies() { return copies; }
extern "C" cudaError_t __real_cudaMemcpyAsync(void*, const void*, std::size_t, cudaMemcpyKind, cudaStream_t);
extern "C" cudaError_t __wrap_cudaMemcpyAsync(void* output, const void* input, std::size_t bytes,
    cudaMemcpyKind kind, cudaStream_t stream) {
  const auto status = __real_cudaMemcpyAsync(output, input, bytes, kind, stream);
  if (fault == ReadFault::None || kind != cudaMemcpyDeviceToHost || status != cudaSuccess ||
      bytes != Parents * sizeof(fe::ShellBatchFailureState)) return status;
  ++copies;
  const auto mode = fault;
  fault = ReadFault::None;
  const auto completed = cudaStreamSynchronize(stream);
  if (completed != cudaSuccess) return completed;
  auto& last = static_cast<fe::ShellBatchFailureState*>(output)[Parents - 1];
  if (mode == ReadFault::InvalidPolicy) {
    last = fe::ShellBatchFailureState::Constant();
  } else if (auto* points = last.tab1_points()) {
    if (mode == ReadFault::NonfiniteDamage) points[2].damage = std::numeric_limits<double>::quiet_NaN();
    if (mode == ReadFault::InvalidDisplayCap) points[2].maximum_damage = 2.;
    if (mode == ReadFault::InvalidFlag) *reinterpret_cast<unsigned char*>(&points[2].point_active) = 2;
  } else return cudaErrorInvalidValue;
  return cudaSuccess;
}
} // namespace resident_tab1_test
