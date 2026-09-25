// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "../Incidence.h"
namespace tlfea::contact::radioss_type25::assembly {
IncidenceStatus DeviceIncidenceBuilder::Impl::Check(const DeviceConnectivity& in) const noexcept {
  if (!usable) return IncidenceStatus::Unusable;
  if (!in.nodes || in.nodes > limits.max_nodes || in.schedule.row_count > limits.max_rows ||
      in.schedule.cohort_count > limits.max_cohorts) return IncidenceStatus::ResourceLimit;
  if ((bool(in.schedule.row_count) != bool(in.schedule.cohort_count)) ||
      in.schedule.cohort_count > in.schedule.row_count || !in.stamp.source ||
      !in.stamp.topology || !in.stamp.associations || !in.stamp.cohorts || !in.stamp.attempt)
    return IncidenceStatus::InvalidInput;
  detail::Range maps, cohorts, allocation;
  if (!detail::RangeOf(in.rows,in.schedule.row_count,maps) ||
      !detail::RangeOf(in.schedule.cohort_ends,in.schedule.cohort_count,cohorts) ||
      !detail::RangeOf(static_cast<const std::byte*>(arena),layout.forecast.device_bytes,allocation) ||
      detail::Overlap(maps,allocation) || detail::Overlap(cohorts,allocation))
    return IncidenceStatus::InvalidInput;
  return IncidenceStatus::Ok;
}
IncidenceStatus DeviceIncidenceBuilder::Stage(const DeviceConnectivity& in) noexcept {
  if (!impl_) return IncidenceStatus::NotInitialized;
  auto& p = *impl_; p.pending = false; p.report = {}; p.report.stamp = in.stamp;
  if (p.sequence == UINT64_MAX) {
    p.usable = false; p.report.status = IncidenceStatus::ResourceLimit; return p.report.status;
  }
  ++p.sequence;
  auto status = p.Check(in);
  if (status != IncidenceStatus::Ok) { p.report.status = status; return status; }
  auto error = cudaGetLastError();
  if (error == cudaSuccess) error = device_detail::Build(p.device,in,p.stream);
  p.report.own_kernel_launches = 1 + (in.schedule.cohort_count ? 1 : 0) +
      (in.schedule.row_count ? 2 : 0);
  p.report.sort_calls = in.schedule.row_count ? 1 : 0;
  if (error == cudaSuccess) error = cudaMemcpyAsync(&p.failure,p.device.failure,sizeof(p.failure),
      cudaMemcpyDeviceToHost,p.stream);
  const auto drained = cudaStreamSynchronize(p.stream); p.report.host_fences = 1;
  if (error != cudaSuccess || drained != cudaSuccess) {
    p.usable = false; status = IncidenceStatus::DeviceFailure;
  } else if (p.failure != ~0ull) {
    const auto index = p.failure & ((1ull<<56)-1);
    if (p.failure >> 56) p.report.bad_occurrence = index;
    else p.report.bad_cohort = index;
    status = IncidenceStatus::InvalidInput;
  }
  p.report.status = status; p.pending = status == IncidenceStatus::Ok;
  if (p.pending) p.current = in;
  return status;
}
DeviceIncidenceView DeviceIncidenceBuilder::view() const noexcept {
  DeviceIncidenceView out;
  if (!impl_ || !impl_->pending) return out;
  const auto& p = *impl_; out.owner_ = p.identity; out.sequence_ = p.sequence;
  out.incidence_ = {p.device.offsets,p.device.occurrences,p.current.nodes,5*p.current.schedule.row_count};
  out.stamp_ = p.current.stamp;
  return out;
}
bool DeviceIncidenceBuilder::IsCurrent(const DeviceIncidenceView& in) const noexcept {
  return impl_ && impl_->pending && in.owner_ == impl_->identity && in.sequence_ == impl_->sequence;
}
void DeviceIncidenceBuilder::Discard() noexcept { if (impl_) impl_->pending = false; }
IncidenceReport DeviceIncidenceBuilder::last_report() const noexcept { return impl_ ? impl_->report : IncidenceReport{}; }
IncidenceForecast DeviceIncidenceBuilder::allocations() const noexcept {
  return impl_ ? impl_->layout.forecast : IncidenceForecast{};
}
} // namespace tlfea::contact::radioss_type25::assembly
