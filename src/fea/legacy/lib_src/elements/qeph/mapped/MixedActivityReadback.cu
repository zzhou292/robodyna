// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../QephBatchStorage.h"
#include "MixedActivityValues.h"
#include <cstring>

namespace tl::fea::qeph {
namespace {
BatchReport MixedError(mapped::MixedActivityError error) {
  using Error = mapped::MixedActivityError;
  if (error == Error::Elastic) return {BatchStatus::NonfiniteResult,"Nonfinite elastic section history"};
  if (error == Error::Plastic) return {BatchStatus::NonfiniteResult,"Nonfinite plastic section history"};
  if (error == Error::OnePoint) return {BatchStatus::InvalidInput,"One-point history is unavailable"};
  return {BatchStatus::InvalidInput,"Unsupported section readback law"};
}
}

BatchReport QephBatch::Impl::ReadMappedMixedActivity(unsigned slab) {
  const auto source = plasticity->CheckActivitySectionSources(slab,config.element_count);
  if (source.status != shell_batch_plasticity_detail::SetupStatus::Success)
    return {BatchStatus::InvalidInput,source.message};
  const auto bytes = mapped::ActivityBytes(config.element_count);
  const auto& packet = device_header.assembly.activity;
  if (slab > 1 || mixed_activity_staging.size() != bytes ||
      !packet.first_invalid || !packet.active || !plasticity->mixed_device())
    return {BatchStatus::InvalidInput,"Mixed section readback shape is invalid"};
  auto report = Runtime(cudaMemsetAsync(packet.first_invalid,0xff,sizeof(std::uint32_t),stream),
      "QEPH mixed activity control initialization failed");
  if (report.status != BatchStatus::Success) return report;
  batch_detail::LaunchMappedMixedActivity(storage,config.element_count,slab,plasticity->mixed_device(),stream);
  report = Runtime(cudaGetLastError(),"QEPH mixed activity validation launch failed");
  if (report.status != BatchStatus::Success) return report;
  report = Runtime(cudaMemcpyAsync(mixed_activity_staging.data(),packet.first_invalid,bytes,
      cudaMemcpyDeviceToHost,stream),"QEPH mixed activity packet readback failed");
  if (report.status != BatchStatus::Success) return report;
  report = Runtime(cudaStreamSynchronize(stream),"QEPH mixed activity readback stream failed");
  if (report.status != BatchStatus::Success) return report;
  std::uint32_t key = UINT32_MAX;
  std::memcpy(&key,mixed_activity_staging.data(),sizeof(key));
  if (key != UINT32_MAX) {
    if (key / mapped::MixedActivityKeyStride >= config.element_count)
      return MixedError(mapped::MixedActivityError::Unsupported);
    return MixedError(static_cast<mapped::MixedActivityError>(key % mapped::MixedActivityKeyStride));
  }
  for (std::size_t parent = 0; parent < config.element_count; ++parent) {
    if (!mapped::ValidMixedActivityRole(MixedActivityRoles()[parent]))
      return MixedError(mapped::MixedActivityError::Unsupported);
  }
  return {BatchStatus::Success,"OK"};
}
} // namespace tl::fea::qeph
