// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Device.h"
namespace tl::fea::physical_activity {
PhysicalActivityReport Decode(const FamilyControl& control, PhysicalActivityFamily family) noexcept {
  if (control.first_error == UINT64_MAX) return {};
  PhysicalActivityReport report;
  report.family = family;
  report.stage = static_cast<PhysicalActivityStage>(control.first_error >> 56);
  report.family_index = (control.first_error >> 16) & 0xffffffffu;
  report.detail = control.first_error & 0xffffu;
  report.status = PhysicalActivityStatus::FamilyFailure;
  report.message = "Physical activity failed the existing typed family predicate";
  if (report.stage == PhysicalActivityStage::Transition) {
    report.status = report.detail == 1 ? PhysicalActivityStatus::Reactivation : PhysicalActivityStatus::InvalidActivity;
    report.message = report.detail == 1 ? "Inactive physical parent cannot reactivate" : "Physical activity is not binary";
  }
  return report;
}
} // namespace tl::fea::physical_activity
