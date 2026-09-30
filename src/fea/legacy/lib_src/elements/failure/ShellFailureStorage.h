#pragma once
#include "ShellFailureArenaLayout.h"
#include "../ShellBatchFailureBinding.h"
#include "../ShellBatchPlasticityStorage.h"

namespace tl::fea::shell_batch_plasticity_detail {
class FailureHostStorage {
 public:
  FailureHostStorage() = default;
  FailureHostStorage(const FailureHostStorage&) = delete;
  FailureHostStorage& operator=(const FailureHostStorage&) = delete;
  ~FailureHostStorage();
  static bool Forecast(std::size_t count, std::size_t binding_bytes, std::size_t device_cap,
      const ShellBatchFailureLimits&,FailureLayout&, std::size_t& host_bytes) noexcept;
  SetupReport Initialize(const ShellBatchFailureBinding&, ShellBindingFamily, std::size_t, const FailureLayout&);
  SetupReport Read(unsigned slab, std::size_t count, cudaStream_t, double time, const ShellBatchLayeredSection*) noexcept;
  SetupReport CheckActivitySources(unsigned slab,std::size_t count) const noexcept;
  SetupReport CheckReadSources(unsigned slab, std::size_t count, const ShellBatchLayeredSection*) const noexcept;
  FailureDeviceStorage* device() const noexcept { return device_; }
  std::size_t device_bytes() const noexcept { return device_ ? layout_.bytes : 0; }
  const ShellBatchFailureBinding& binding() const noexcept { return binding_; }
  const ShellBatchFailureState* staging() const noexcept { return staging_.data(); }
 private:
  friend class HostStorage;
  ShellBatchFailureBinding binding_;
  FailureDeviceStorage* device_ = nullptr;
  FailureDeviceStorage header_;
  FailureLayout layout_;
  ShellBindingFamily family_ = ShellBindingFamily::None;
  std::size_t count_ = 0;
  util::BoundedStartupArray<ShellBatchFailureState, 0> staging_;
};
} // namespace tl::fea::shell_batch_plasticity_detail
