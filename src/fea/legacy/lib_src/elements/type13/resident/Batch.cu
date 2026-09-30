// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include <new>

namespace tl::fea::type13 {
Batch::Batch() = default;
Batch::~Batch() = default;
Batch::Impl::~Impl() {
  if (device) {
    cudaFree(device);
  }
}
BatchReport Batch::Impl::Runtime(cudaError_t error, const char* message) noexcept {
  if (error == cudaSuccess) {
    return {};
  }
  usable = false;
  Discard();
  return {BatchStatus::DeviceFailure, message};
}
BatchReport Batch::Impl::PendingError() noexcept {
  if (!usable) {
    return {BatchStatus::Unusable, "TYPE13 CUDA storage is poisoned"};
  }
  return Runtime(cudaGetLastError(), "Pending CUDA error before TYPE13 operation");
}
BatchReport Batch::Impl::ReadControl() {
  auto report = Runtime(cudaGetLastError(), "TYPE13 kernel launch failed");
  if (!report) {
    return report;
  }
  report = Runtime(cudaMemcpyAsync(&control, &device->control, sizeof(control),
      cudaMemcpyDeviceToHost, stream), "TYPE13 control readback failed");
  if (!report) {
    return report;
  }
  report = Runtime(cudaStreamSynchronize(stream), "TYPE13 owner stream failed");
  if (!report) {
    return report;
  }
  return {control.status,
          control.status == BatchStatus::Success ? "OK" : "TYPE13 device validation rejected",
          control.element, control.node, control.element_status};
}

BatchReport Batch::InitializeJoined(const BatchConfig& config,
                                    const Type13NodeContributions& source) try {
  if (impl_) {
    return {BatchStatus::InvalidInput, "TYPE13 batch is already initialized"};
  }
  BatchForecast forecast;
  auto report = Forecast(config, source, forecast);
  if (!report) {
    return report;
  }
  batch_detail::ArenaLayout layout;
  if (!batch_detail::MakeLayout(source.model()->property_count(),
      source.model()->connection_count(), config.limits, layout)) {
    return {BatchStatus::ResourceLimit, "TYPE13 arena layout is inconsistent"};
  }
  util::HostArena arena;
  if (!arena.Initialize(layout.bytes)) {
    return {BatchStatus::ResourceLimit, "TYPE13 startup arena allocation failed"};
  }
  batch_detail::Storage header;
  BatchDiagnostics diagnostics;
  report = batch_detail::BuildStartup(config, source, arena, layout, header, diagnostics);
  if (!report) {
    return report;
  }
  auto next = std::make_unique<Impl>(config, source);
  next->layout = layout;
  next->host_bytes = forecast.startup_host_bytes;
  next->accepted_diagnostics = diagnostics;
  next->staging = std::make_unique<Evaluation[]>(source.model()->connection_count());
  report = next->Upload(arena, header);
  if (!report) return report;
  impl_ = std::move(next);
  return {};
} catch (const std::bad_alloc&) {
  return {BatchStatus::ResourceLimit, "TYPE13 host allocation failed"};
}

void Batch::DiscardTrial() noexcept {
  if (impl_) {
    impl_->Discard();
  }
}
NodalAllocationInfo Batch::allocations() const noexcept {
  return impl_ ? NodalAllocationInfo{impl_->layout.bytes, 1} : NodalAllocationInfo{};
}
std::size_t Batch::startup_host_bytes() const noexcept {
  return impl_ ? impl_->host_bytes : 0;
}
} // namespace tl::fea::type13
