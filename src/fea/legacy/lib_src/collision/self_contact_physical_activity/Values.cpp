// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"

namespace tlfea::contact::self_contact_physical_activity {

namespace {
SelfContactPhysicalActivityReport Failure(
    SelfContactPhysicalActivityStatus status, std::size_t parent,
    const char* message) noexcept {
  SelfContactPhysicalActivityReport report;
  report.status = status;
  report.parent = parent;
  report.message = message;
  return report;
}
}  // namespace

SelfContactPhysicalActivityReport ValidateTransition(
    const std::uint8_t* base, const std::uint8_t* current,
    std::size_t count) noexcept {
  if (!base || !current || !count)
    return Failure(SelfContactPhysicalActivityStatus::InvalidInput,
                   SIZE_MAX,
                   "Activity transition buffers are missing");
  for (std::size_t parent = 0; parent < count; ++parent) {
    if (base[parent] > 1 || current[parent] > 1)
      return Failure(SelfContactPhysicalActivityStatus::InvalidActivity,
                     parent,
                     "Physical parent activity is not binary");
    if (current[parent] > base[parent])
      return Failure(SelfContactPhysicalActivityStatus::Reactivation,
                     parent,
                     "Inactive self-contact parent cannot reactivate");
  }
  return {};
}

}  // namespace tlfea::contact::self_contact_physical_activity
