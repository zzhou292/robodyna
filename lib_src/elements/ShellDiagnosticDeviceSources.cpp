// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ShellDiagnosticDeviceSources.h"
#include "ShellMixedSectionStorage.h"
#include "failure/ShellFailureStorage.h"
namespace tl::fea::shell_batch_plasticity_detail {
bool HostStorage::DiagnosticSources(DiagnosticDeviceSources& output) const noexcept {
  if(one_point_||!element_count_||(!mixed_&&!device_))return false;
  DiagnosticDeviceSources next;next.parent_count=element_count_;
  if(mixed_) {
    next.mixed_device=mixed_->device_;next.mixed=mixed_->header_;
    next.curve_points=mixed_->layout_.curve_x.count;
    if(!next.mixed_device||mixed_->layout_.curve_y.count!=next.curve_points)return false;
  } else {
    next.plain_device=device_;next.plain=device_header_;next.curve_points=layout_.curve_x.count;
    if(layout_.curve_y.count!=next.curve_points)return false;
  }
  if(failure_) {next.failure_device=failure_->device_;next.failure=failure_->header_;}
  output=next;return true;
}
} // namespace tl::fea::shell_batch_plasticity_detail
