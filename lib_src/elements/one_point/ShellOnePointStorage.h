#pragma once
#include "ShellOnePointArenaLayout.h"
#include "../ShellBatchPlasticityStorage.h"

namespace tl::fea::shell_batch_plasticity_detail {
class OnePointHostStorage {
 public:
  OnePointHostStorage() = default;
  ~OnePointHostStorage();
  OnePointHostStorage(const OnePointHostStorage&) = delete;
  OnePointHostStorage& operator=(const OnePointHostStorage&) = delete;
  static bool Forecast(std::size_t count, std::size_t device_cap, std::size_t host_cap,
      OnePointLayout&, std::size_t& host_bytes) noexcept;
  SetupReport Initialize(const ShellBatchPlasticityBinding&, const ShellBatchBinding&,
      std::size_t count, const OnePointLayout&);
  SetupReport Read(unsigned slab, std::size_t count, cudaStream_t,
      const ShellBatchPlasticityBinding&, double time) noexcept;
  OnePointDeviceStorage* device() const noexcept { return device_; }
  std::size_t device_bytes() const noexcept { return device_ ? layout_.bytes : 0; }
  const ShellBatchOnePointSectionState* staging() const noexcept { return staging_.data(); }
 private:
  OnePointDeviceStorage* device_ = nullptr;
  OnePointDeviceStorage header_;
  OnePointLayout layout_;
  std::size_t count_ = 0;
  util::BoundedStartupArray<ShellBatchOnePointSectionState, 0> staging_;
};
} // namespace tl::fea::shell_batch_plasticity_detail
