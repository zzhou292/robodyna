// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../ShellBatchPublicationValues.h"

namespace tl::fea::shell_publication_detail {
template<class Report>
ShellPublicationReport PhysicalReport(const Report& report) noexcept {
  using Status = decltype(report.status);
  if (report.status == Status::Success) return Ok();
  if (report.status == Status::DeviceFailure || report.nodal_status == NodalStatus::DeviceFailure)
    return {S::DeviceFailure,report.message,report.nodal_status};
  if (report.status == Status::ResourceLimit)
    return {S::ResourceLimit,report.message,report.nodal_status};
  if (report.status == Status::NotInitialized)
    return {S::NotInitialized,report.message,report.nodal_status};
  return {S::NotJoined,report.message,report.nodal_status};
}
bool CompletePhysicalParticipants(const ShellPhysicalBinding&,
    const ShellPhysicalParticipants&) noexcept;
} // namespace tl::fea::shell_publication_detail
