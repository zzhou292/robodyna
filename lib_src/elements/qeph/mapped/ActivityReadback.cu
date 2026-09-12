// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../QephBatchStorage.h"
#include "ActivityValues.h"
#include <cstring>

namespace tl::fea::qeph {
BatchReport QephBatch::Impl::ValidateMappedActivity(unsigned slab) {
  if (!physical) return {BatchStatus::Success,"OK"};
  auto report = PendingError();
  if (report.status != BatchStatus::Success) return report;
  const auto bytes = mapped::ActivityBytes(config.element_count);
  const auto& packet = device_header.assembly.activity;
  if (slab > 1 || activity_staging.size() != bytes ||
      !packet.first_invalid || !packet.active || !plasticity->mixed_device()) {
    return {BatchStatus::InvalidInput,"Mapped Qeph activity readback shape is missing"};
  }
  const bool accepted_slab = slab == AcceptedSlabIndex();
  const auto epoch = accepted_stamp.epoch + (accepted_slab ? 0 : 1);
  const auto time = accepted_stamp.time + (accepted_slab ? 0 : config.owner.fixed_dt);
  report = Runtime(cudaMemsetAsync(packet.first_invalid,0xff,sizeof(std::uint32_t),stream),
      "QEPH activity control initialization failed");
  if (report.status != BatchStatus::Success) return report;
  batch_detail::LaunchMappedActivity(storage,config.element_count,slab,time,epoch,
      plasticity->mixed_device(),stream);
  report = Runtime(cudaGetLastError(),"QEPH activity validation launch failed");
  if (report.status != BatchStatus::Success) return report;
  report = Runtime(cudaMemcpyAsync(activity_staging.data(),packet.first_invalid,bytes,
      cudaMemcpyDeviceToHost,stream),"QEPH activity packet readback failed");
  if (report.status != BatchStatus::Success) return report;
  report = Runtime(cudaStreamSynchronize(stream),"QEPH activity readback stream failed");
  if (report.status != BatchStatus::Success) return report;
  std::uint32_t first_invalid = mapped::NoActivityFailure;
  std::memcpy(&first_invalid,activity_staging.data(),sizeof(first_invalid));
  const auto* active = activity_staging.data() + sizeof(first_invalid);
  if (first_invalid != mapped::NoActivityFailure) {
    return {BatchStatus::NonfiniteResult,"Mapped Qeph force cache differs from its source/endpoint role",
        first_invalid};
  }
  const auto* failure_active = FailureActivity();
  return mapped::ValidateRoleActivity(*physical->catalog(),MixedActivityRoles(),
      config.element_count,[active](std::size_t parent) { return active[parent]; },
      [failure_active](std::size_t parent) { return failure_active[parent]; });
}
} // namespace tl::fea::qeph
