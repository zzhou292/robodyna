// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../QephBatchStorage.h"
#include "ActivityValues.h"
#include "../../failure/ShellFailureReadback.h"
#include <cstring>

namespace tl::fea::qeph {
BatchReport QephBatch::Impl::ReadParentActivity(unsigned slab, double time) {
  if (!physical) {
    return shell_batch_plasticity_detail::ReadFailure(*this,slab,time,
        [this](unsigned selected) { return ValidateMappedActivity(selected); });
  }
  auto report = PendingError();
  if (report.status != BatchStatus::Success) return report;
  report = ReadMappedMixedActivity(slab);
  if (report.status != BatchStatus::Success) return report;
  const auto sections = plasticity->CheckActivityFailureSources(slab,config.element_count);
  using Setup = shell_batch_plasticity_detail::SetupStatus;
  if (sections.status == Setup::DeviceFailure) return Runtime(sections.cuda_status,sections.message);
  if (sections.status != Setup::Success) {
    return {sections.status == Setup::NonfiniteResult ? BatchStatus::NonfiniteResult : BatchStatus::InvalidInput,
        sections.message};
  }
  report = ReadMappedFailureActivity(slab,time);
  if (report.status != BatchStatus::Success) return report;
  // The failure packet is now retained in separate host storage. Only after
  // that synchronized success may force validation reuse the device packet.
  return ValidateMappedActivity(slab);
}

BatchReport QephBatch::Impl::ReadMappedFailureActivity(unsigned slab, double time) {
  auto report = PendingError();
  if (report.status != BatchStatus::Success) return report;
  const auto bytes = mapped::ActivityBytes(config.element_count);
  const auto& packet = device_header.assembly.activity;
  if (slab > 1 || failure_activity_staging.size() != bytes ||
      !packet.first_invalid || !packet.active || !plasticity->mixed_device() || !plasticity->failure_device()) {
    return {BatchStatus::InvalidInput,"Mapped Qeph failure activity readback shape is missing"};
  }
  report = Runtime(cudaMemsetAsync(packet.first_invalid,0xff,sizeof(std::uint32_t),stream),
      "QEPH failure activity control initialization failed");
  if (report.status != BatchStatus::Success) return report;
  batch_detail::LaunchMappedFailureActivity(storage,config.element_count,slab,time,
      plasticity->mixed_device(),plasticity->failure_device(),stream);
  report = Runtime(cudaGetLastError(),"QEPH failure activity validation launch failed");
  if (report.status != BatchStatus::Success) return report;
  report = Runtime(cudaMemcpyAsync(failure_activity_staging.data(),packet.first_invalid,bytes,
      cudaMemcpyDeviceToHost,stream),"QEPH failure activity packet readback failed");
  if (report.status != BatchStatus::Success) return report;
  report = Runtime(cudaStreamSynchronize(stream),"QEPH failure activity readback stream failed");
  if (report.status != BatchStatus::Success) return report;
  std::uint32_t first_invalid = mapped::NoActivityFailure;
  std::memcpy(&first_invalid,failure_activity_staging.data(),sizeof(first_invalid));
  if (first_invalid != mapped::NoActivityFailure) {
    // Full failure Read reports this phase without a public parent index.
    return {BatchStatus::NonfiniteResult,"Failure sidecar state disagrees with its declared policy/saved section"};
  }
  // Preserve rejection of corrupted readback encodings before a flag is used
  // as bool or copied to the caller. The device produced only exact zero/one.
  for (std::size_t parent = 0; parent < config.element_count; ++parent) {
    if (FailureActivity()[parent] > 1) {
      return {BatchStatus::NonfiniteResult,"Failure sidecar state disagrees with its declared policy/saved section"};
    }
  }
  return {BatchStatus::Success,"OK"};
}
} // namespace tl::fea::qeph
