#include "T3BatchStorage.h"
#include <new>

namespace tl::fea::t3 {
BatchReport T3Batch::InitializeJoined(const T3BatchConfig& config,
    const ShellBatchBinding& binding, const ShellBatchPlasticityBinding& catalog,
    const ShellBatchFailureBinding& failure, const ShellBatchFailureLimits& limits) {
  if (!catalog.Matches(binding) || !failure.Matches(catalog)) {
    return {BatchStatus::InvalidInput,
            "Failure declaration differs from the complete joined section catalog"};
  }
  return InitializeImpl(config, nullptr, &binding, nullptr, failure.catalog(), nullptr, &failure, &limits);
}

BatchReport T3Batch::Impl::InitializeFailure(const ShellBatchFailureBinding& failure,
    const ShellBatchFailureLimits& limits) {
  using namespace shell_batch_plasticity_detail;
  if (!joined_binding) {
    return {BatchStatus::InvalidInput, "Failure requires complete joined scope"};
  }
  std::unique_ptr<HostStorage> next(new(std::nothrow) HostStorage);
  if (!next) return {BatchStatus::ResourceLimit, "Failure host allocation failed"};
  const auto setup = next->InitializeFailureCollection(failure, *joined_binding,
      ShellBindingFamily::T3, config.element_count, config.max_device_bytes - layout.bytes,
      config.storage_limits.max_host_bytes, limits, VehicleShellResidentLimits(config.storage_limits));
  if (setup.status == SetupStatus::DeviceFailure) return Runtime(setup.cuda_status, setup.message);
  if (setup.status != SetupStatus::Success) {
    return {setup.status == SetupStatus::ResourceLimit ? BatchStatus::ResourceLimit : BatchStatus::InvalidInput,
            setup.message};
  }
  plasticity = std::move(next);
  return {BatchStatus::Success, "OK"};
}
} // namespace tl::fea::t3
