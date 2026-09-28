// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
namespace tl::fea::physical_activity {
PhysicalActivityReport State::Capture(const NodalTrialToken& trial, const NodalAssemblyView* a,
    const NodalPreparedView* p, const ShellPhysicalDiagnostics& d) noexcept {
  using S = PhysicalActivityStatus;
  auto report = GuardOtherFamilies(physical, d);
  if (report.status != S::Ok) return report;
  QephInput q; T3Input t;
  if (forecast.qeph_count) {
    report = BatchAccess::Borrow(*participants.qeph, *owner, *publication, physical, trial, a, p, d.qeph, q);
    if (report.status != S::Ok) return report;
  }
  if (forecast.t3_count) {
    report = BatchAccess::Borrow(*participants.t3, *owner, *publication, physical, trial, a, p, d.t3, t);
    if (report.status != S::Ok) return report;
  }
  if (q.count != forecast.qeph_count || t.count != forecast.t3_count)
    return {S::SourceMismatch, "Activity batch counts differ from their source"};
  auto* controls = host_control;
  controls[0] = {}; controls[1] = {};
  const auto runtime = [&](cudaError_t error) {
    if (error == cudaSuccess) return true;
    usable = false; return false;
  };
  if (!runtime(cudaMemcpyAsync(device_control, controls, sizeof(host_control), cudaMemcpyHostToDevice, stream)))
    return {S::DeviceFailure, "Activity control initialization failed"};
  if (q.count && !runtime(physical_activity::Capture(q,
      {laws, p ? base : nullptr, staging, device_control}, stream)))
    return {S::DeviceFailure, "QEPH activity validation launch failed"};
  const auto offset = forecast.qeph_count;
  if (t.count && !runtime(physical_activity::Capture(t,
      {laws + offset, p ? base + offset : nullptr, staging + offset, device_control + 1}, stream)))
    return {S::DeviceFailure, "T3 activity validation launch failed"};
  if (!runtime(cudaMemcpyAsync(controls, device_control, sizeof(host_control), cudaMemcpyDeviceToHost, stream)) ||
      !runtime(cudaStreamSynchronize(stream)))
    return {S::DeviceFailure, "Physical activity scalar readback failed"};
  report = Decode(controls[0], PhysicalActivityFamily::Qeph);
  if (report.status == S::Ok) report = Decode(controls[1], PhysicalActivityFamily::T3);
  if (report.status != S::Ok) return report;
  for (unsigned family = 0; family < 2; ++family) {
    const auto count = family ? forecast.t3_count : forecast.qeph_count;
    const auto& c = controls[family];
    if (c.active > count || c.removed > count ||
        ((c.active == count) != (c.first_inactive == UINT32_MAX)) ||
        ((c.removed == 0) != (c.first_removed == UINT32_MAX)) ||
        (c.first_inactive != UINT32_MAX && c.first_inactive >= count) ||
        (c.first_removed != UINT32_MAX && c.first_removed >= count))
      return {S::InvalidActivity, "Physical activity scalar packet is invalid"};
  }
  // No device writes or other fallible work after this observer's publication.
  for (unsigned family = 0; family < 2; ++family) {
    const auto& c = controls[family];
    summaries[family] = {family ? forecast.t3_count : forecast.qeph_count, c.active,
        c.first_inactive == UINT32_MAX ? SIZE_MAX : c.first_inactive, c.removed,
        c.first_removed == UINT32_MAX ? SIZE_MAX : c.first_removed};
  }
  return {};
}
} // namespace tl::fea::physical_activity
