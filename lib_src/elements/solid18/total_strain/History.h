// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ForceChecks.h"
namespace tl::fea::solid18::total_strain {
namespace force_detail {
struct HistoryWriter {
  // Output is unpublished caller scratch, disjoint from every borrowed input.
  TL_SOLID18_HD static Status Prepare(const Reference& r,const Material& m,
      const HistoryValues& values,HistoryStamp stamp,History& output) noexcept {
    if(!ValidMaterial(r,m)||!ValidValues(m,values)||!tl::math::Finite(stamp.time_s)||
       stamp.time_s<0||(stamp.sample_index==0&&stamp.time_s!=0))return Status::InvalidInput;
    output.reference_=r;output.material_=m;output.values_=values;output.stamp_=stamp;
    output.prepared_=true;
    return Status::Success;
  }
};
}
// Finite prescribed values, not a restart reader or a new clock. Borrowed
// immutable material curves must outlive the history, including on device.
inline Status PreparePrescribedHistory90(const Reference& r,const Material& m,
    const HistoryValues& values,HistoryStamp stamp,History& output) noexcept {
  History next;
  const auto status=force_detail::HistoryWriter::Prepare(r,m,values,stamp,next);
  if(status==Status::Success)output=next;
  return status;
}
} // namespace tl::fea::solid18::total_strain
