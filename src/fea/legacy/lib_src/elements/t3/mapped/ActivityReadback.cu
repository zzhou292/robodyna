// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ActivityQuery.h"
#include <cstring>
namespace tl::fea::t3 {
namespace {
using Phase = mapped::ActivityPhase;
BatchReport RoleAgreement(const ShellBatchPlasticityBinding& catalog, std::size_t count,
    const std::uint8_t* roles, const std::uint8_t* force, const std::uint8_t* failure) noexcept {
  for (std::size_t parent=0;parent<count;++parent) {
    ShellSectionLaw law=ShellSectionLaw::Unspecified;
    if (!catalog.Law(ShellBindingFamily::T3,parent,&law) || roles[parent]!=static_cast<std::uint8_t>(law))
      return {BatchStatus::NonfiniteResult,"Mapped T3 typed section role differs",static_cast<std::uint32_t>(parent)};
    if (force[parent]!=(failure[parent]?1:0))
      return {BatchStatus::NonfiniteResult,"Mapped T3 force and failure activity differ",static_cast<std::uint32_t>(parent)};
  }
  return {BatchStatus::Success,"OK"};
}
}
bool T3Batch::Impl::CompactActivityEligible(unsigned slab) const noexcept {
  const auto bytes = mapped_shell::ActivityBytes(config.element_count);
  return physical && joined_binding && plasticity && storage && config.element_count &&
      config.element_count < UINT32_MAX/mapped::ActivityKeyStride &&
      activity_staging.size()==bytes && activity_roles.size()==bytes && activity_failure.size()==bytes &&
      device_header.assembly.activity.first_invalid && device_header.assembly.activity.active &&
      plasticity->SupportsCompactActivity(slab,config.element_count,ShellBindingFamily::T3);
}
BatchReport T3Batch::Impl::ReadCompactActivity(unsigned slab,double time,std::uint64_t epoch) {
  auto report=PendingError();
  if (report.status!=BatchStatus::Success) return report;
  const bool accepted=slab==AcceptedSlabIndex();
  mapped::ActivityQuery query{storage,plasticity->mixed_device(),plasticity->failure_device(),
      plasticity->one_point_device(),config.element_count,slab,time,
      accepted_stamp.time+(accepted?0:config.owner.fixed_dt),epoch,accepted_stamp.epoch+(accepted?0:1),
      plasticity->section_catalog()->execution_sections()};
  const auto bytes=mapped_shell::ActivityBytes(config.element_count);
  const auto& packet=device_header.assembly.activity;
  const auto phase=[&](Phase which, std::uint8_t* destination) -> BatchReport {
    auto checked=Runtime(cudaMemsetAsync(packet.first_invalid,0xff,sizeof(std::uint32_t),stream),
        "T3 compact activity control initialization failed");
    if (checked.status!=BatchStatus::Success) return checked;
    checked=Runtime(mapped::LaunchActivity(query,which,stream),"T3 compact activity validation launch failed");
    if (checked.status!=BatchStatus::Success) return checked;
    checked=Runtime(cudaMemcpyAsync(destination,packet.first_invalid,bytes,cudaMemcpyDeviceToHost,stream),
        "T3 compact activity packet readback failed");
    if (checked.status!=BatchStatus::Success) return checked;
    checked=Runtime(cudaStreamSynchronize(stream),"T3 compact activity readback stream failed");
    if (checked.status!=BatchStatus::Success) return checked;
    std::uint32_t key=mapped::NoActivityFailure;
    std::memcpy(&key,destination,sizeof(key));
    checked=mapped::ActivityErrorReport(which,key,config.element_count);
    if (checked.status!=BatchStatus::Success) return checked;
    if (which==Phase::Force || which==Phase::Failure) {
      for (std::size_t parent=0;parent<config.element_count;++parent) if (destination[4+parent]>1)
        return mapped::ActivityErrorReport(which,mapped::ActivityKey(static_cast<std::uint32_t>(parent),
            which==Phase::Force?mapped::ActivityError::ForceResult:mapped::ActivityError::FailureState),config.element_count);
    }
    return {BatchStatus::Success,"OK"};
  };
  if (query.point) {
    report=phase(Phase::OnePoint,activity_staging.data());
    if (report.status!=BatchStatus::Success) return report;
  }
  report=phase(Phase::Mixed,activity_roles.data());
  if (report.status!=BatchStatus::Success) return report;
  report=phase(Phase::Failure,activity_failure.data());
  if (report.status!=BatchStatus::Success) return report;
  // The old mapped force readback owns this pending-error boundary.
  report=PendingError();
  if (report.status!=BatchStatus::Success) return report;
  report=phase(Phase::Force,activity_staging.data());
  if (report.status!=BatchStatus::Success) return report;
  report=RoleAgreement(*physical->catalog(),config.element_count,activity_roles.data()+4,
      activity_staging.data()+4,activity_failure.data()+4);
  if (report.status!=BatchStatus::Success || !query.point) return report;
  // Preserve the source preflight and second pending-error check of the
  // existing within-call force-copy reuse, then validate point/force identity.
  if (!plasticity || !plasticity->one_point_sections() || !joined_binding || slab>1)
    return {BatchStatus::InvalidInput,"One-point readback has no complete T3 source scope"};
  if (!plasticity->section_catalog()) return {BatchStatus::InvalidInput,"One-point readback catalog is unavailable"};
  report=PendingError();
  return report.status==BatchStatus::Success ? phase(Phase::PointIdentity,activity_staging.data()) : report;
}
} // namespace tl::fea::t3
