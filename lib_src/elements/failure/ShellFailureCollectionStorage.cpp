#include "ShellFailureStorage.h"
#include "../ShellMixedSectionStorage.h"
#include "../one_point/ShellOnePointStorage.h"
#include "../ShellResidentHostAccounting.h"
#include <new>

namespace tl::fea::shell_batch_plasticity_detail {
bool HostStorage::ForecastFailureSections(std::size_t count, std::size_t points,
    std::size_t binding_bytes, std::size_t device_cap, std::size_t host_cap,
    const ShellBatchFailureLimits& limits, std::size_t& host_bytes, bool one_point) noexcept {
  FailureLayout failure;
  MixedLayout mixed;
  OnePointLayout point;
  std::size_t point_host = 0;
  std::size_t failure_host = 0, mixed_host = 0;
  if ((one_point && !OnePointHostStorage::Forecast(count, device_cap, host_cap, point, point_host)) ||
      !FailureHostStorage::Forecast(count, binding_bytes, device_cap - point.bytes, limits, failure, failure_host) ||
      !MixedHostStorage::Forecast(count, points, 0, device_cap - point.bytes - failure.bytes,
                                 host_cap, mixed, mixed_host)) {
    return false;
  }
  util::BoundedArenaLayout host(host_cap);
  util::ArenaRegion ignored;
  if (!host.Append<unsigned char>(failure_host, ignored) ||
      !host.Append<unsigned char>(mixed_host, ignored) ||
      (one_point && !host.Append<unsigned char>(point_host, ignored))) {
    return false;
  }
  host_bytes = host.bytes();
  return true;
}

SetupReport HostStorage::InitializeFailureCollection(const ShellBatchFailureBinding& failure,
    const ShellBatchBinding& binding, ShellBindingFamily family, std::size_t count,
    std::size_t device_cap, std::size_t host_cap,
    const ShellBatchFailureLimits& limits, bool vehicle) {
  const auto* catalog = failure.catalog();
  if (device_ || mixed_ || collection_ || failure_ || !catalog || !catalog->Matches(binding) ||
      (family != ShellBindingFamily::Qeph && family != ShellBindingFamily::T3) ||
      count != (family == ShellBindingFamily::Qeph ? binding.qeph_count() : binding.t3_count())) {
    return {SetupStatus::InvalidInput, "Failure requires a complete joined mixed catalog"};
  }
  std::size_t native_bytes = 0, catalog_bytes = 0;
  if (!shell_batch_detail::RetainedScopeBytes(&binding, catalog, vehicle, native_bytes, catalog_bytes) ||
      failure.host_bytes() < catalog->host_bytes()) {
    return {SetupStatus::ResourceLimit, "Invalid retained failure payload"};
  }
  const auto failure_bytes = failure.host_bytes() - catalog->host_bytes() + catalog_bytes;
  ShellSectionCounts counts;
  if (!catalog->Counts(family, &counts)) {
    return {SetupStatus::InvalidInput, "Failure catalog family is unavailable"};
  }
  const bool one_point = counts.law44_nip1 != 0;
  if (one_point && family != ShellBindingFamily::T3) {
    return {SetupStatus::InvalidInput, "One-point resident state is T3-only"};
  }
  std::size_t host_bytes = 0, unused = 0;
  FailureLayout failure_layout;
  MixedLayout mixed_layout;
  OnePointLayout point_layout;
  if (!ForecastFailureSections(count, catalog->curve_point_count(), failure_bytes,
                               device_cap, host_cap, limits, host_bytes, one_point) ||
      (one_point && !point_layout.Initialize(count, device_cap)) ||
      !FailureHostStorage::Forecast(count, failure_bytes, device_cap - point_layout.bytes,
                                   limits, failure_layout, unused) ||
      !mixed_layout.Initialize(count, catalog->curve_point_count(),
                               device_cap - point_layout.bytes - failure_layout.bytes)) {
    return {SetupStatus::ResourceLimit, "Mixed failure collection exceeds explicit budgets"};
  }
  std::unique_ptr<FailureHostStorage> failure_storage(new(std::nothrow) FailureHostStorage);
  std::unique_ptr<MixedHostStorage> mixed_storage(new(std::nothrow) MixedHostStorage);
  std::unique_ptr<OnePointHostStorage> point_storage;
  if (one_point) point_storage.reset(new(std::nothrow) OnePointHostStorage);
  if (!failure_storage || !mixed_storage || (one_point && !point_storage)) {
    return {SetupStatus::ResourceLimit, "Mixed failure host allocation failed"};
  }
  auto result = failure_storage->Initialize(failure, family, count, failure_layout);
  if (result.status != SetupStatus::Success) return result;
  result = mixed_storage->Initialize(*failure_storage->binding().catalog(), family, count, mixed_layout);
  if (result.status != SetupStatus::Success) return result;
  if (point_storage) {
    result = point_storage->Initialize(*catalog, binding, count, point_layout);
    if (result.status != SetupStatus::Success) return result;
  }
  failure_ = std::move(failure_storage);
  mixed_ = std::move(mixed_storage);
  one_point_ = std::move(point_storage);
  element_count_ = count;
  return {SetupStatus::Success, "OK"};
}

FailureDeviceStorage* HostStorage::failure_device() const noexcept {
  return failure_ ? failure_->device() : nullptr;
}
const ShellBatchFailureState* HostStorage::failure_staging() const noexcept {
  return failure_ ? failure_->staging() : nullptr;
}
std::size_t HostStorage::failure_device_bytes() const noexcept {
  return failure_ ? failure_->device_bytes() : 0;
}
bool HostStorage::SameFailureScope(const HostStorage& other) const noexcept {
  return (!failure_ && !other.failure_) ||
      (failure_ && other.failure_ && failure_->binding().SameScope(other.failure_->binding()));
}
const ShellBatchPlasticityBinding* HostStorage::Collection() const noexcept {
  return failure_ ? failure_->binding().catalog() : collection_.get();
}
} // namespace tl::fea::shell_batch_plasticity_detail
