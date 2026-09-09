#include "PrescribedShellBatch.h"

#include <chrono>
#include <cstdint>
#include <cstring>
#include <limits>

namespace tl::qualification::reissner_batch {
namespace {
using Clock = std::chrono::steady_clock;
double Milliseconds(Clock::time_point begin) {
  return std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
}
__global__ void EvaluatePrescribed(const PrescribedInput* input, shell::ShellResult* result,
                                   shell::ShellStatus* status, unsigned count) {
  const unsigned i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= count) return;
  status[i] = shell::ComputeShellForce(input[i].reference, input[i].section,
                                      input[i].configuration, result[i]);
}
__global__ void ReduceStatuses(const shell::ShellStatus* status, unsigned* aggregate, unsigned count) {
  if (blockIdx.x || threadIdx.x) return;
  unsigned failures = 0, first = count;
  for (unsigned i = 0; i < count; ++i) {
    if (status[i] != shell::ShellStatus::kSuccess) {
      ++failures;
      if (first == count) first = i;
    }
  }
  aggregate[0] = failures;
  aggregate[1] = first;
}
bool Disjoint(const void* first, std::size_t first_size, const void* second, std::size_t second_size) {
  const auto a = reinterpret_cast<std::uintptr_t>(first), b = reinterpret_cast<std::uintptr_t>(second);
  const auto maximum = std::numeric_limits<std::uintptr_t>::max();
  if (first_size > maximum - a || second_size > maximum - b) return false;
  return a + first_size <= b || b + second_size <= a;
}
}  // namespace

bool PrescribedShellBatch::AdmittedCount(unsigned count) {
  return count == 2 || count == 8 || count == 32 || count == 128;
}
std::size_t PrescribedShellBatch::DeviceBytes(unsigned count) {
  if (!AdmittedCount(count)) return 0;
  return count * (sizeof(PrescribedInput) + sizeof(shell::ShellResult) + sizeof(shell::ShellStatus)) + sizeof(Aggregate);
}
PrescribedShellBatch::~PrescribedShellBatch() {
  // No exception or implicit recovery. CUDA failures poison the owner, and
  // destruction only releases resources that this object actually acquired.
  if (stream_) cudaStreamSynchronize(stream_);
  if (event_stop_) cudaEventDestroy(event_stop_);
  if (event_start_) cudaEventDestroy(event_start_);
  if (device_allocation_) cudaFree(device_allocation_);
  if (stream_) cudaStreamDestroy(stream_);
}
Report PrescribedShellBatch::CudaFailure(cudaError_t error, const char* operation) {
  poisoned_ = true;
  have_statuses_ = false;
  poison_report_ = {Status::kCudaFailure, operation, error};
  return poison_report_;
}
Report PrescribedShellBatch::Ready() const {
  if (poisoned_) return poison_report_;
  if (!initialized_) return {Status::kNotInitialized, "Initialize must succeed first"};
  return {};
}

Report PrescribedShellBatch::Initialize(unsigned count) {
  if (poisoned_) return poison_report_;
  if (initialized_) return {Status::kAlreadyInitialized, "single initialization"};
  if (!AdmittedCount(count)) return {Status::kInvalidCapacity, "only 2/8/32/128 elements"};
  Initialization value;
  const std::size_t allocation_bytes = DeviceBytes(count);
  value.owned_host_bytes = sizeof(*this);
  const auto first = Clock::now();
#define P1_CUDA(call) do { const cudaError_t error = (call); if (error != cudaSuccess) return CudaFailure(error, #call); } while (false)
  P1_CUDA(cudaMemGetInfo(&value.after_context_query.free_bytes, &value.after_context_query.total_bytes));
  value.first_memory_query_ms = Milliseconds(first);
  initialization_ = value;
  P1_CUDA(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking));
  P1_CUDA(cudaEventCreate(&event_start_));
  P1_CUDA(cudaEventCreate(&event_stop_));
  P1_CUDA(cudaMalloc(&device_allocation_, allocation_bytes));
  value.owned_device_bytes = allocation_bytes;
  initialization_ = value;  // Retained allocation remains accounted on later failure.
  auto* bytes = static_cast<unsigned char*>(device_allocation_);
  device_inputs_ = reinterpret_cast<PrescribedInput*>(bytes);
  bytes += count * sizeof(PrescribedInput);
  device_results_ = reinterpret_cast<shell::ShellResult*>(bytes);
  bytes += count * sizeof(shell::ShellResult);
  device_statuses_ = reinterpret_cast<shell::ShellStatus*>(bytes);
  bytes += count * sizeof(shell::ShellStatus);
  device_aggregate_ = reinterpret_cast<Aggregate*>(bytes);
  P1_CUDA(cudaMemsetAsync(device_allocation_, 0, value.owned_device_bytes, stream_));
  P1_CUDA(cudaStreamSynchronize(stream_));
  P1_CUDA(cudaMemGetInfo(&value.after_owned_allocation.free_bytes, &value.after_owned_allocation.total_bytes));
  initialization_ = value;
  cudaFuncAttributes attributes{};
  P1_CUDA(cudaFuncGetAttributes(&attributes, EvaluatePrescribed));
  auto& kernel = value.kernel;
  kernel.registers_per_thread = attributes.numRegs;
  kernel.local_bytes_per_thread = attributes.localSizeBytes;
  kernel.static_shared_bytes = attributes.sharedSizeBytes;
  kernel.maximum_threads_per_block = attributes.maxThreadsPerBlock;
  kernel.binary_version = attributes.binaryVersion;
  kernel.ptx_version = attributes.ptxVersion;
  P1_CUDA(cudaDeviceGetLimit(&kernel.current_stack_limit_bytes, cudaLimitStackSize));
  int device = 0;
  P1_CUDA(cudaGetDevice(&device));
  cudaDeviceProp properties{};
  P1_CUDA(cudaGetDeviceProperties(&properties, device));
  kernel.multiprocessors = properties.multiProcessorCount;
  kernel.maximum_threads_per_multiprocessor = properties.maxThreadsPerMultiProcessor;
  P1_CUDA(cudaOccupancyMaxActiveBlocksPerMultiprocessor(
      &kernel.estimated_active_blocks_per_multiprocessor, EvaluatePrescribed, kThreadsPerBlock, 0));
  kernel.estimated_occupancy = static_cast<double>(kernel.estimated_active_blocks_per_multiprocessor * kThreadsPerBlock) /
                               properties.maxThreadsPerMultiProcessor;
  P1_CUDA(cudaMemGetInfo(&value.after_kernel_introspection.free_bytes, &value.after_kernel_introspection.total_bytes));
  initialization_ = value;
  count_ = count;
  initialized_ = true;
  return {};
}

Report PrescribedShellBatch::UploadElement(unsigned index, const PrescribedInput& input) {
  const auto ready = Ready();
  if (ready.status != Status::kSuccess) return ready;
  if (index >= count_) return {Status::kInvalidArgument, "upload index outside initialized capacity"};
  have_statuses_ = false;
  P1_CUDA(cudaMemcpyAsync(device_inputs_ + index, &input, sizeof(input), cudaMemcpyHostToDevice, stream_));
  P1_CUDA(cudaStreamSynchronize(stream_));  // Caller can reuse its one host input.
  uploaded_[index] = true;
  return {};
}

Report PrescribedShellBatch::EvaluateChecked(shell::ShellResult* output, unsigned count, EvaluationTiming& timing) {
  const auto ready = Ready();
  if (ready.status != Status::kSuccess) return ready;
  have_statuses_ = false;
  if (!output || count != count_ || !Disjoint(output, count * sizeof(*output), &timing, sizeof(timing)) ||
      !Disjoint(output, count * sizeof(*output), this, sizeof(*this)) ||
      !Disjoint(&timing, sizeof(timing), this, sizeof(*this)))
    return {Status::kInvalidArgument, "independent output/timing storage of exact initialized capacity required"};
  for (unsigned i = 0; i < count_; ++i)
    if (!uploaded_[i]) return {Status::kMissingInput, "every prescribed slot must be uploaded", cudaSuccess, i};
  if (evaluations_ == kMaximumEvaluations) return {Status::kEvaluationLimit, "lifetime evaluation cap is 100"};
  ++evaluations_;  // Uploads and rejected physical configurations never reset it.
  const auto begin = Clock::now();
  P1_CUDA(cudaGetLastError());
  P1_CUDA(cudaEventRecord(event_start_, stream_));
  EvaluatePrescribed<<<(count_ + kThreadsPerBlock - 1) / kThreadsPerBlock, kThreadsPerBlock, 0, stream_>>>(
      device_inputs_, device_results_, device_statuses_, count_);
  P1_CUDA(cudaGetLastError());
  P1_CUDA(cudaEventRecord(event_stop_, stream_));
  ReduceStatuses<<<1, 1, 0, stream_>>>(device_statuses_, reinterpret_cast<unsigned*>(device_aggregate_), count_);
  P1_CUDA(cudaGetLastError());
  P1_CUDA(cudaMemcpyAsync(&host_aggregate_, device_aggregate_, sizeof(Aggregate), cudaMemcpyDeviceToHost, stream_));
  P1_CUDA(cudaMemcpyAsync(host_statuses_, device_statuses_, count_ * sizeof(shell::ShellStatus), cudaMemcpyDeviceToHost, stream_));
  P1_CUDA(cudaMemcpyAsync(host_results_, device_results_, count_ * sizeof(shell::ShellResult), cudaMemcpyDeviceToHost, stream_));
  P1_CUDA(cudaStreamSynchronize(stream_));
  EvaluationTiming candidate;
  float kernel_ms = 0;
  P1_CUDA(cudaEventElapsedTime(&kernel_ms, event_start_, event_stop_));
  candidate.force_kernel_ms = kernel_ms;
  P1_CUDA(cudaMemGetInfo(&candidate.after_completion.free_bytes, &candidate.after_completion.total_bytes));
  have_statuses_ = true;
  // Check both the device reduction and every independent status lane. Neither
  // a successful first element nor stale result bytes imply aggregate success.
  unsigned failures = 0, first_failure = count_;
  for (unsigned i = 0; i < count_; ++i) {
    if (host_statuses_[i] != shell::ShellStatus::kSuccess) {
      ++failures;
      if (first_failure == count_) first_failure = i;
    }
  }
  if (host_aggregate_.failures != failures || host_aggregate_.first_failure != first_failure)
    return CudaFailure(cudaErrorUnknown, "device/host aggregate status disagreement");
  if (failures)
    return {Status::kElementFailure, "prescribed element rejected", cudaSuccess, first_failure, host_statuses_[first_failure]};
  std::memcpy(output, host_results_, count_ * sizeof(shell::ShellResult));
  candidate.checked_end_to_end_ms = Milliseconds(begin);
  timing = candidate;
  return {};
#undef P1_CUDA
}
}  // namespace tl::qualification::reissner_batch
