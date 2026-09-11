// SPDX-License-Identifier: AGPL-3.0-or-later
// Adapted from the retained QEPH native history adapter, (C) 2026 Siemens.
#pragma once
#include "QephHistoryData.h"
#include "QephKinematics.h"

namespace tl::fea::qeph::detail {
TL_QEPH_HD inline bool SameHistoryBits(double a,double b) {
  // Character access is legal object-representation inspection on host/device.
  const auto* x=reinterpret_cast<const unsigned char*>(&a);
  const auto* y=reinterpret_cast<const unsigned char*>(&b);
  for(unsigned i=0;i<sizeof(double);++i) if(x[i]!=y[i]) return false;
  return true;
}
template<unsigned N> TL_QEPH_HD inline bool FiniteHistoryArray(const double (&a)[N]) {
  for(double value:a) if(!tl::math::Finite(value)) return false;
  return true;
}
TL_QEPH_HD inline bool ValidHistoryValues(const HistoryValues& h,bool allow_inactive=false) {
  return FiniteHistoryArray(h.stress)&&FiniteHistoryArray(h.material_stress)&&
      FiniteHistoryArray(h.bending_stress)&&FiniteHistoryArray(h.stabilization)&&
      FiniteHistoryArray(h.strain_curvature)&&FiniteHistoryArray(h.internal_work)&&
      Positive(h.thickness)&&tl::math::Finite(h.hourglass_viscous_work)&&(h.active==1||(allow_inactive&&h.active==0));
}
} // namespace tl::fea::qeph::detail

namespace tl::fea::qeph {
TL_QEPH_HD inline bool History::matches_reference(const ReferenceData& r) const noexcept {
  if(!prepared_||!r.prepared) return false;
  const auto& a=r.input; const auto& b=reference_input_;
  for(unsigned n=0;n<4;++n)
    if(a.node_ids[n]!=b.node_ids[n]||
       !detail::SameHistoryBits(a.position[n].x,b.position[n].x)||
       !detail::SameHistoryBits(a.position[n].y,b.position[n].y)||
       !detail::SameHistoryBits(a.position[n].z,b.position[n].z)) return false;
  return detail::SameHistoryBits(a.density,b.density)&&
      detail::SameHistoryBits(a.young_modulus,b.young_modulus)&&
      detail::SameHistoryBits(a.poisson_ratio,b.poisson_ratio)&&
      detail::SameHistoryBits(a.thickness,b.thickness);
}

// Explicit prescribed finite values, not a native restart admission. Failure
// preserves output bytes, including when values alias an existing output.
TL_QEPH_HD inline Status History::Prepare(const ReferenceData& r,
    const HistoryValues& values,HistoryStamp stamp,History& output,bool allow_inactive) noexcept {
  if(!detail::SaneReference(r)) return Status::kInvalidReference;
  if(!detail::ValidHistoryValues(values,allow_inactive)||!tl::math::Finite(stamp.time)||stamp.time<0)
    return Status::kInvalidInput;
  History candidate;
  candidate.data_=values; candidate.stamp_=stamp;
  candidate.reference_input_=r.input; candidate.prepared_=true;
  output=candidate;
  return Status::kSuccess;
}
TL_QEPH_HD inline Status PreparePrescribedHistory(const ReferenceData& r,
    const HistoryValues& values,HistoryStamp stamp,History& output) noexcept {
  return History::Prepare(r,values,stamp,output,false);
}
// Explicit accepted OFF0/1 value path; not native restart or owner admission.
TL_QEPH_HD inline Status PrepareFailurePrescribedHistory(const ReferenceData& r,
    const HistoryValues& values,HistoryStamp stamp,History& output) noexcept {
  return History::Prepare(r,values,stamp,output,true);
}
TL_QEPH_HD inline Status InitializeHistory(const ReferenceData& r,
    HistoryStamp stamp,History& output) noexcept {
  HistoryValues initial;
  initial.thickness=r.input.thickness;
  return PreparePrescribedHistory(r,initial,stamp,output);
}
} // namespace tl::fea::qeph
