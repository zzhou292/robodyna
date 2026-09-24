// SPDX-License-Identifier: AGPL-3.0-or-later
#include "RepresentedIntervalCrossingGpu.h"
#include "represented_interval_crossing/native_device/Workspace.h"
#include "represented_interval_crossing/BusyRelease.h"
#include "represented_interval_crossing/DeviceBatch.h"
#include <new>
#include <utility>

namespace tlfea::contact {
namespace device = represented_interval_crossing::native_device;
struct RepresentedIntervalCrossingGpu::Impl {
  RepresentedIntervalCrossing native;
  device::Workspace workspace;
  RepresentedIntervalGpuForecast forecast;
  std::atomic<bool> busy{false};
};
RepresentedIntervalCrossingGpu::RepresentedIntervalCrossingGpu() noexcept = default;
RepresentedIntervalCrossingGpu::~RepresentedIntervalCrossingGpu() = default;

RepresentedIntervalGpuPreflight RepresentedIntervalCrossingGpu::Preflight(
    RepresentedIntervalGpuLimits limits) noexcept {
  device::Layout layout;
  return device::MakeLayout(limits, sizeof(RepresentedIntervalCrossingGpu) + sizeof(Impl), layout);
}
RepresentedIntervalGpuReport RepresentedIntervalCrossingGpu::Initialize(
    RepresentedIntervalGpuLimits limits, cudaStream_t stream) noexcept try {
  if (impl_) {
    RepresentedIntervalGpuReport result;
    result.native.status = RepresentedIntervalStatus::AlreadyInitialized;
    result.native.message = "Native CUDA owner is already initialized";
    return result;
  }
  device::Layout layout;
  auto checked = device::MakeLayout(limits, sizeof(*this) + sizeof(Impl), layout);
  if (checked.report.native.status != RepresentedIntervalStatus::Ok) return checked.report;
  auto next = std::make_unique<Impl>();
  checked.report.native = next->native.Initialize(limits.native);
  if (checked.report.native.status != RepresentedIntervalStatus::Ok) {
    checked.report.device = {};
    return checked.report;
  }
  const auto initialized = next->workspace.Initialize(layout, stream,
      this, sizeof(*this), next.get(), sizeof(*next));
  if (initialized.native.status != RepresentedIntervalStatus::Ok) return initialized;
  next->forecast = checked.forecast;
  impl_ = std::move(next);
  return initialized;
} catch (const std::bad_alloc&) {
  RepresentedIntervalGpuReport result;
  result.native.status = RepresentedIntervalStatus::ResourceLimit;
  result.native.message = "Native CUDA owner allocation failed";
  result.device = {RepresentedIntervalDeviceStatus::ResourceLimit, result.native.message};
  return result;
}
RepresentedIntervalGpuReport RepresentedIntervalCrossingGpu::Certify(
    const RepresentedTrianglePath* paths, std::size_t path_count,
    const RepresentedTrianglePair* pairs, std::size_t pair_count, cudaStream_t stream) noexcept {
  RepresentedIntervalGpuReport result;
  if (!impl_) {
    result.native.status = RepresentedIntervalStatus::NotInitialized;
    result.native.message = "Native CUDA owner is not initialized";
    return result;
  }
  bool idle = false;
  if (!impl_->busy.compare_exchange_strong(idle, true, std::memory_order_acq_rel)) {
    result.native.status = RepresentedIntervalStatus::InvalidInput;
    result.native.input_paths = path_count;
    result.native.input_pairs = pair_count;
    result.native.message = "Native CUDA owner does not accept concurrent calls";
    return result;
  }
  represented_interval_crossing::BusyRelease release{&impl_->busy};
  result.native = impl_->workspace.BeginAttempt(stream);
  if (result.native.status == RepresentedIntervalStatus::Ok)
    result.native = represented_interval_crossing::DeviceAccess::Certify(
        impl_->native, paths, path_count, pairs, pair_count, impl_->workspace);
  else {
    result.native.input_paths = path_count;
    result.native.input_pairs = pair_count;
  }
  result.device = impl_->workspace.report();
  return result;
}
bool RepresentedIntervalCrossingGpu::initialized() const noexcept { return bool(impl_); }
RepresentedIntervalGpuForecast RepresentedIntervalCrossingGpu::forecast() const noexcept {
  return impl_ ? impl_->forecast : RepresentedIntervalGpuForecast{};
}
RepresentedIntervalResultView RepresentedIntervalCrossingGpu::results() const noexcept {
  return impl_ && !impl_->busy.load(std::memory_order_acquire)
      ? impl_->native.results() : RepresentedIntervalResultView{};
}

represented_interval_crossing::DeviceBatchReport
represented_interval_crossing::DeviceBatchAccess::Certify(
    RepresentedIntervalCrossingGpu& crossing,
    const RepresentedTrianglePath* paths, std::size_t path_count,
    const RepresentedTrianglePair* pairs, std::size_t pair_count,
    std::size_t batch_pair_capacity, RepresentedIntervalResult* scratch,
    std::size_t scratch_capacity, cudaStream_t stream) noexcept {
  DeviceBatchReport result;
  if (!crossing.impl_) {
    result.native.status = RepresentedIntervalStatus::NotInitialized;
    result.native.message = "Crossing batch owner is not initialized";
    return result;
  }
  auto& owner = *crossing.impl_;
  bool idle = false;
  if (!owner.busy.compare_exchange_strong(idle, true, std::memory_order_acq_rel)) {
    result.native.status = RepresentedIntervalStatus::InvalidInput;
    result.native.message = "Native CUDA owner does not accept concurrent calls";
    return result;
  }
  BusyRelease release{&owner.busy};
  const auto admission = owner.workspace.BeginAttempt(stream);
  if (admission.status != RepresentedIntervalStatus::Ok) {
    result.native.status = admission.status;
    result.native.message = admission.message;
  } else {
    result.native = DeviceAccess::CertifyBatch(owner.native, paths, path_count,
        pairs, pair_count, batch_pair_capacity, scratch, scratch_capacity, owner.workspace);
  }
  result.device = owner.workspace.report();
  return result;
}
}  // namespace tlfea::contact
