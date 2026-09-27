// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../QbatBatchStorage.h"
#include "Values.h"
#include <cstring>
namespace tl::fea::qbat {
BatchReport Batch::Impl::ReadParentActivity(const batch_detail::Slab* source,
    const BatchDiagnostics& expected) {
  auto report=PendingError();
  if(report.status!=BatchStatus::Success) return report;
  const unsigned slab=source==&storage->slab[0]?0u:1u;
  if(source!=&storage->slab[slab]) return {BatchStatus::InvalidInput,"Unknown QBAT result slab"};
  const auto& packet=device_header.activity;
  const auto bytes=mapped_shell::ActivityBytes(config.element_count);
  if (!config.element_count || config.element_count>=UINT32_MAX || activity_staging.size()!=bytes ||
      !packet.first_invalid || !packet.active) {
    return {BatchStatus::InvalidInput,"QBAT activity packet shape is missing"};
  }
  report=Runtime(cudaMemsetAsync(packet.first_invalid,0xff,sizeof(std::uint32_t),stream),
      "QBAT activity control initialization failed");
  if(report.status!=BatchStatus::Success) return report;
  batch_detail::LaunchParentActivity(storage,config.element_count,slab,expected.time,expected.epoch,stream);
  report=Runtime(cudaGetLastError(),"QBAT activity validation launch failed");
  if(report.status!=BatchStatus::Success) return report;
  report=Runtime(cudaMemcpyAsync(activity_staging.data(),packet.first_invalid,bytes,
      cudaMemcpyDeviceToHost,stream),"QBAT activity packet readback failed");
  if(report.status!=BatchStatus::Success) return report;
  report=Runtime(cudaStreamSynchronize(stream),"QBAT activity packet stream failed");
  if(report.status!=BatchStatus::Success) return report;
  std::uint32_t first_invalid=activity::NoInvalidParent;
  std::memcpy(&first_invalid,activity_staging.data(),sizeof(first_invalid));
  report=activity::Complete(first_invalid,ParentActivity(),config.element_count,[&](std::size_t parent) {
    // Uploaded materials were built/rebased from this immutable catalog. Keep
    // its original lookup/error obligation while the complete result predicate
    // reads those same source parameters and curve values on the GPU.
    Material material;
    return Failure().catalog()->Parameters(ShellBindingFamily::Qbat,parent,&material);
  });
  if(report.status!=BatchStatus::Success) Discard();
  return report;
}
} // namespace tl::fea::qbat
