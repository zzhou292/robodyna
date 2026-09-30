// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Values.h"
#include <memory>
#include <stdexcept>
namespace tl::fea::cin_capture_test {
namespace {
void Check(cudaError_t error) {
  if (error != cudaSuccess) throw std::runtime_error(cudaGetErrorString(error));
}
struct Free { void operator()(void* value) const noexcept { cudaFree(value); } };
class DeviceValues {
 public:
  explicit DeviceValues(Values& values) : input(values.Input()) {
    input.control = static_cast<nodal_detail::Control*>(Upload(&values.control, sizeof(values.control)));
    input.work = static_cast<double*>(Upload(values.work.data(), values.work.size()*sizeof(double)));
    input.capture.node = static_cast<double*>(Upload(values.output.data(), values.output.size()*sizeof(double)));
    input.capture.node_rotation = input.capture.node+3*values.nodes;
    input.groups.member_nodes = static_cast<std::uint8_t*>(Upload(values.members.data(), values.members.size()));
  }
  void Read(Values& values) const {
    Check(cudaMemcpy(values.output.data(), input.capture.node, values.output.size()*sizeof(double), cudaMemcpyDeviceToHost));
    Check(cudaMemcpy(values.work.data(), input.work, values.work.size()*sizeof(double), cudaMemcpyDeviceToHost));
  }
  cin_advance::Input input;
 private:
  void* Upload(const void* source, std::size_t bytes) {
    void* pointer = nullptr;
    Check(cudaMalloc(&pointer, bytes));
    storage.emplace_back(pointer);
    Check(cudaMemcpy(pointer, source, bytes, cudaMemcpyHostToDevice));
    return pointer;
  }
  std::vector<std::unique_ptr<void, Free>> storage;
};
__global__ void Frozen(const cin_advance::Input input) {
  if (input.control->status != NodalStatus::Ok) return;
  CopyFrozen(input);
}
}
TEST(CinParallelCaptureCuda, CompleteFrozenCopyMatchesEveryBitAcrossBlocksAndRigidRows) {
  for (const auto n : {1u, 127u, 128u, 129u, 272u, 33001u}) {
    for (bool masks : {false, true}) {
      SCOPED_TRACE(n);
      Values old(n), next(n);
      DeviceValues serial(old), parallel(next);
      if (!masks) serial.input.groups.member_nodes = parallel.input.groups.member_nodes = nullptr;
      Frozen<<<1,1>>>(serial.input);
      Check(cudaGetLastError());
      Check(cin_advance::capture::Launch(parallel.input, nullptr));
      Check(cudaStreamSynchronize(nullptr));
      serial.Read(old);
      parallel.Read(next);
      SameValues(old.output, next.output);
      SameValues(old.work, next.work);
    }
  }
}
TEST(CinParallelCaptureCuda, FailedSuffixAndDisabledCaptureConsumeNothingThenSameAllocationRetry) {
  Values values(33001), expected(33001);
  values.control.status = NodalStatus::InvalidOutput;
  DeviceValues device(values);
  const auto input = device.input;
  // Even a nonnull capture must not read invalid work/mask after suffix failure.
  device.input.work = nullptr;
  device.input.groups.member_nodes = reinterpret_cast<const std::uint8_t*>(1);
  Check(cin_advance::capture::Launch(device.input, nullptr));
  Check(cudaStreamSynchronize(nullptr));
  device.input = input;
  device.Read(values);
  SameValues(values.output, expected.output);
  device.input.capture.node = nullptr;
  device.input.control = nullptr;
  device.input.work = nullptr;
  Check(cin_advance::capture::Launch(device.input, nullptr));
  device.input = input;
  values.control.status = NodalStatus::Ok;
  Check(cudaMemcpy(device.input.control, &values.control, sizeof(values.control), cudaMemcpyHostToDevice));
  Check(cin_advance::capture::Launch(device.input, nullptr));
  Check(cudaStreamSynchronize(nullptr));
  device.Read(values);
  CopyFrozen(expected.Input());
  SameValues(values.output, expected.output);
}
} // namespace tl::fea::cin_capture_test
