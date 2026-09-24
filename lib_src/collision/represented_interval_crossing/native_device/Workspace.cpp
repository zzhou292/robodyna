// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Workspace.h"
#include "../NativeStorageDomain.h"
#include "../native/Identity.h"
#include "lib_src/solvers/NodalTrialIdentity.h"

namespace tlfea::contact::represented_interval_crossing::native_device {
namespace {
RepresentedIntervalReport Failure(RepresentedIntervalStatus status, const char* message) noexcept {
  RepresentedIntervalReport result;
  result.status = status;
  result.message = message;
  return result;
}
bool ExplicitStream(cudaStream_t stream) noexcept {
  return stream && stream != cudaStreamLegacy && stream != cudaStreamPerThread;
}
}  // namespace
Workspace::~Workspace() { if (device_) cudaFree(device_); }

RepresentedIntervalGpuReport Workspace::Initialize(const Layout& layout, cudaStream_t stream,
    const void* facade, std::size_t facade_bytes, const void* owner, std::size_t owner_bytes) noexcept {
  RepresentedIntervalGpuReport result;
  if (!ExplicitStream(stream)) {
    result.native = Failure(RepresentedIntervalStatus::InvalidInput,
        "Native CUDA execution requires an explicit stream");
    result.device = {RepresentedIntervalDeviceStatus::InvalidInput, result.native.message};
    return result;
  }
  unsigned flags = 0;
  auto error = cudaStreamGetFlags(stream, &flags);
  if (error == cudaSuccess) error = cudaGetDevice(&device_ordinal_);
  if (error != cudaSuccess) {
    result.native = DeviceFailure(error);
    result.device = report_;
    return result;
  }
  layout_ = layout;
  stream_ = stream;
  facade_ = facade;
  facade_bytes_ = facade_bytes;
  owner_ = owner;
  owner_bytes_ = owner_bytes;
  if (!host_.Initialize(layout.host_payload_bytes) ||
      !(jobs_ = host_.Construct<DeviceJob>(layout.host_jobs)) ||
      !(results_ = host_.Construct<DeviceResult>(layout.host_results))) {
    result.native = Failure(RepresentedIntervalStatus::ResourceLimit, "Native CUDA host staging allocation failed");
    result.device = {RepresentedIntervalDeviceStatus::ResourceLimit, result.native.message};
    return result;
  }
  if (layout.host_cache.count && !(cache_ = host_.Construct<CachedPair>(layout.host_cache))) {
    result.native = Failure(RepresentedIntervalStatus::ResourceLimit,
        "Native CUDA numerical cache construction failed");
    result.device = {RepresentedIntervalDeviceStatus::ResourceLimit, result.native.message};
    return result;
  }
  error = cudaMalloc(&device_, layout.forecast.device_bytes);
  if (error != cudaSuccess) {
    result.native = DeviceFailure(error);
    result.device = report_;
    return result;
  }
  usable_ = true;
  report_ = {RepresentedIntervalDeviceStatus::Ok, "OK"};
  result.device = report_;
  return result;
}

RepresentedIntervalReport Workspace::DeviceFailure(cudaError_t error) noexcept {
  usable_ = false;
  report_.status = RepresentedIntervalDeviceStatus::DeviceFailure;
  report_.message = cudaGetErrorString(error);
  return Failure(RepresentedIntervalStatus::ResourceLimit, report_.message);
}
RepresentedIntervalReport Workspace::BeginAttempt(cudaStream_t stream) noexcept {
  report_ = {};
  if (!usable_) {
    report_ = {RepresentedIntervalDeviceStatus::DeviceFailure, "Native CUDA owner is poisoned"};
    return Failure(RepresentedIntervalStatus::ResourceLimit, report_.message);
  }
  int current = -1;
  const auto error = cudaGetDevice(&current);
  if (error != cudaSuccess) return DeviceFailure(error);
  if (stream != stream_ || current != device_ordinal_) {
    report_ = {RepresentedIntervalDeviceStatus::InvalidInput, "Native CUDA stream or device changed"};
    return Failure(RepresentedIntervalStatus::InvalidInput, report_.message);
  }
  return {};
}
bool Workspace::Disjoint(const void* data, std::size_t bytes) const noexcept {
  if (!bytes) return true;
  using tl::fea::trial_identity::Disjoint;
  return Disjoint(data, bytes, facade_, facade_bytes_) &&
      Disjoint(data, bytes, owner_, owner_bytes_) &&
      Disjoint(data, bytes, this, sizeof(*this)) &&
      Disjoint(data, bytes, host_.data(), host_.bytes()) &&
      Disjoint(data, bytes, device_, layout_.forecast.device_bytes);
}
RepresentedIntervalReport Workspace::Execute(const AuthenticatedWork& work) noexcept {
  // The native owner authenticated all inputs and all staging capacities first.
  // This independent disjoint check covers the new workspace, including output
  // buffers, before any upload or host-staging write can read aliases.
  if (work.scene_.executor_ != this ||
      !Disjoint(work.paths(), work.path_count() * sizeof(*work.paths())) ||
      !Disjoint(work.pairs(), work.pair_count() * sizeof(*work.pairs())) ||
      !Disjoint(work.staging(), work.pair_count() * sizeof(*work.staging())) ||
      !Disjoint(work.status(), work.pair_count() * sizeof(*work.status()))) {
    report_.status = RepresentedIntervalDeviceStatus::InvalidInput;
    report_.message = "Native CUDA work aliases its workspace or has a foreign scene lease";
    return Failure(RepresentedIntervalStatus::InvalidInput, report_.message);
  }
  if (work.path_count() > layout_.paths.count || work.pair_count() > layout_.jobs.count ||
      work.limits().max_depth + 1 != layout_.dfs_capacity) {
    report_.status = RepresentedIntervalDeviceStatus::ResourceLimit;
    report_.message = "Native CUDA lexical workload exceeds its retained layout";
    return Failure(RepresentedIntervalStatus::ResourceLimit, report_.message);
  }
  report_.status = RepresentedIntervalDeviceStatus::Ok;
  report_.message = "OK";
  return work.scene_.cohort_ ? ExecuteCohort(work) : ExecuteSlice(work);
}
RepresentedIntervalReport Workspace::ExecuteSlice(const AuthenticatedWork& work) noexcept {
  std::size_t jobs = 0;
  for (std::size_t i = 0; i < work.pair_count(); ++i) {
    const auto& pair = work.pairs()[i];
    if (NativeStorageDomain::FromPaths(work.paths()[pair.first], work.paths()[pair.second],
                                     work.limits().max_depth).eligible()) {
      jobs_[jobs++] = {pair, i};
    } else {
      ++report_.host_pairs;
    }
  }
  report_.device_pairs += jobs;
  const auto submitted = Submit(work, jobs);
  if (submitted.status != RepresentedIntervalStatus::Ok) return submitted;
  for (std::size_t i = 0; i < jobs; ++i) {
    const auto ordinal = jobs_[i].ordinal;
    native::StoreResult(results_[i].value, work.staging() + ordinal);
    work.status()[ordinal].complete = true;
    ++report_.consumed_device_pairs;
  }
  return {};
}
}  // namespace tlfea::contact::represented_interval_crossing::native_device
