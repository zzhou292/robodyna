// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "QbatForceChecks.h"

namespace tl::fea::qbat {
// Explicit prescribed finite history, not a native restart/owner admission.
// Curve backing remains immutable caller-owned host/device storage.
TL_QBAT_HD inline Status PreparePrescribedHistory(const Reference& reference,const Material& material,
    Failure failure,const HistoryValues& values,HistoryStamp stamp,History& output) noexcept {
  if (!detail::ValidMaterial(reference,material,failure) ||
      !tl::math::Finite(stamp.time) || stamp.time<0 ||
      !detail::ValidHistory(values,material,stamp.time)) return Status::kInvalidInput;
  History next;
  next.reference_=reference;
  next.material_=material;
  next.failure_=failure;
  next.data_=values;
  next.stamp_=stamp;
  next.prepared_=true;
  output=next;
  return Status::kSuccess;
}
TL_QBAT_HD inline Status InitializeHistory(const Reference& reference,const Material& material,
    Failure failure,HistoryStamp stamp,History& output) noexcept {
  HistoryValues values;
  values.thickness_m=reference.input().quadrilateral.thickness;
  return PreparePrescribedHistory(reference,material,failure,values,stamp,output);
}
} // namespace tl::fea::qbat
