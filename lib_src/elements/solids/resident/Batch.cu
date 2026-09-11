// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include <new>

namespace tl::fea::solids {
Batch::Batch() = default;
Batch::~Batch() = default;
Batch::Impl::~Impl() {
  if (device) cudaFree(device);
}
BatchReport Batch::Impl::Runtime(cudaError_t error, const char* message) noexcept {
  if (error == cudaSuccess) return {};
  usable = false;
  Discard();
  return {BatchStatus::DeviceFailure, message};
}
BatchReport Batch::Impl::PendingError() noexcept {
  if (!usable) return {BatchStatus::Unusable, "Solid device storage is poisoned"};
  return Runtime(cudaGetLastError(), "Pending CUDA error before solid operation");
}
BatchReport Batch::Impl::ReadControl() {
  auto report = Runtime(cudaGetLastError(), "Solid kernel launch failed");
  if (!report) return report;
  report = Runtime(cudaMemcpyAsync(&control, &device->control, sizeof(control),
      cudaMemcpyDeviceToHost, stream), "Solid control readback failed");
  if (!report) return report;
  report = Runtime(cudaStreamSynchronize(stream), "Solid owner stream failed");
  if (!report) return report;
  return {control.status, control.status == BatchStatus::Success ? "OK" : "Solid device validation rejected",
      control.family, control.parent, control.node, control.element_status};
}
BatchReport Batch::InitializeJoined(const BatchConfig& config, const Model& model) try {
  if (impl_) return {BatchStatus::InvalidInput, "Solid batch is already initialized"};
  BatchForecast forecast;
  auto report = Forecast(config, model, forecast);
  if (!report) return report;
  batch_detail::ArenaLayout layout;
  report = batch_detail::Plan(config, model, layout);
  if (!report) return report;
  util::HostArena upload;
  if (!upload.Initialize(layout.bytes))
    return {BatchStatus::ResourceLimit, "Solid upload allocation failed"};
  batch_detail::Storage host_header;
  report = batch_detail::BuildUpload(config, model, upload, layout, host_header);
  if (!report) return report;
  auto next = std::make_unique<Impl>(config, model);
  next->layout = layout;
  next->host_bytes = forecast.startup_host_bytes;
  if (!next->staging.Initialize(layout.staging_bytes))
    return {BatchStatus::ResourceLimit, "Solid readback staging allocation failed"};
  if (!next->staging.Construct<batch_detail::State<batch_detail::Traits18>>(layout.solid18.staging) ||
      !next->staging.Construct<batch_detail::State<batch_detail::Traits24>>(layout.solid24.staging) ||
      !next->staging.Construct<batch_detail::State<batch_detail::Traits6z>>(layout.solid6z.staging) ||
      !next->staging.Construct<batch_detail::State<batch_detail::Traits18Law44>>(layout.solid18_law44.staging) ||
      !next->staging.Construct<batch_detail::State<batch_detail::Traits18Law90>>(layout.solid18_law90.staging))
    return {BatchStatus::ResourceLimit, "Solid staging lifetime construction failed"};
  report = next->PendingError();
  if (!report) return report;
  report = next->Runtime(cudaMalloc(reinterpret_cast<void**>(&next->device), layout.bytes),
      "Solid device arena allocation failed");
  if (!report) return report;
  report = batch_detail::RebaseCurves(model, layout, next->device, host_header);
  if (!report) return report;
  auto device_header = batch_detail::RebasedHeader(next->device, layout);
  device_header.config = config;
  device_header.source_instance_id = model.source_instance_id();
  next->device_header = device_header;
  *util::ArenaPointer<batch_detail::Storage>(upload.data(), layout.header) = device_header;
  report = next->Runtime(cudaMemcpy(next->device, upload.data(), layout.bytes, cudaMemcpyHostToDevice),
      "Solid model upload failed");
  if (!report) return report;
  batch_detail::LaunchInitialize(next->device, next->stream);
  report = next->ReadControl();
  if (!report) return report;
  next->accepted_diagnostics = next->control.diagnostics;
  report = next->ReadResults(0, next->accepted_diagnostics);
  if (!report) return report;
  impl_ = std::move(next);
  return {};
} catch (const std::bad_alloc&) {
  return {BatchStatus::ResourceLimit, "Solid host allocation failed"};
}
void Batch::DiscardTrial() noexcept {
  if (impl_) impl_->Discard();
}
NodalAllocationInfo Batch::allocations() const noexcept {
  return impl_ ? NodalAllocationInfo{impl_->layout.bytes, 1} : NodalAllocationInfo{};
}
std::size_t Batch::startup_host_bytes() const noexcept {
  return impl_ ? impl_->host_bytes : 0;
}
} // namespace tl::fea::solids
