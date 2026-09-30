#include "ResidentFixture.h"

namespace t3_one_point_resident_test {
namespace {
ReadFault armed = ReadFault::None;
unsigned copies = 0;
}
void Arm(ReadFault fault) { armed = fault; copies = 0; }
unsigned FaultCopies() { return copies; }
extern "C" cudaError_t __real_cudaMemcpyAsync(void*,const void*,std::size_t,cudaMemcpyKind,cudaStream_t);
extern "C" cudaError_t __wrap_cudaMemcpyAsync(void* output, const void* input, std::size_t bytes,
    cudaMemcpyKind kind, cudaStream_t stream) {
  const auto result = __real_cudaMemcpyAsync(output, input, bytes, kind, stream);
  if (armed == ReadFault::None || kind != cudaMemcpyDeviceToHost || result != cudaSuccess) return result;
  ++copies;
  const bool shell = armed == ReadFault::ShellMismatch;
  const auto expected = Parents * (shell ? sizeof(t3::ForceTrial) : sizeof(fe::ShellBatchOnePointSectionState));
  if (bytes != expected) return result;
  const auto mode = armed;
  armed = ReadFault::None;
  const auto completed = cudaStreamSynchronize(stream);
  if (completed != cudaSuccess) return completed;
  if (mode == ReadFault::DeviceError) return cudaErrorInvalidValue;
  if (shell) {
    auto& force = static_cast<t3::ForceTrial*>(output)[Parents - 1];
    // Change a valid finite history field without changing the actual device.
    // PreparePrescribed is not used: preserve the reference and stamp precisely.
    const_cast<t3::HistoryValues&>(force.proposed_history.data()).thickness *= 1.001;
  } else {
    auto& point = static_cast<fe::ShellBatchOnePointSectionState*>(output)[Parents - 1];
    if (mode == ReadFault::InvalidFlag)
      *reinterpret_cast<unsigned char*>(&point.point.failure.failed_now) = 2;
    else point.point.current.plastic_work_density = std::numeric_limits<double>::quiet_NaN();
  }
  return cudaSuccess;
}
} // namespace t3_one_point_resident_test
