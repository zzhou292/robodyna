// SPDX-License-Identifier: AGPL-3.0-or-later
// I25GAPM reader-phase exterior and two-support internal EightSlot branches.
#pragma once
#include "MainSolid.h"
namespace tlfea::contact::radioss_type25 {
// Explicit signed reader-volume entry. The legacy ordinary API remains
// positive-only; source membership, volume and phase are caller obligations.
TL_MATH_HOST_DEVICE inline CoefficientStatus EvaluateNativeReaderSolidMainCoefficient(
    const NativeSolidMainCoefficientInput& in,NativeSolidMainCoefficientResult* out) {
  if(in.layout!=SolidLayout::EightSlot)return CoefficientStatus::UnsupportedProfile;
  return coefficient_detail::SolidMainContribution(in,out,
      coefficient_detail::VolumeDomain::SignedNonzero);
}
struct NativeInternalSolidMainCoefficientInput {
  NativeSolidMainCoefficientInput first; // face Internal; actual first support's ICONTR.
  double second_fill=0,second_bulk=0,second_volume=0; // Original second PM32; no invented second ICONTR.
};
TL_MATH_HOST_DEVICE inline CoefficientStatus EvaluateNativeInternalSolidMainCoefficient(
    const NativeInternalSolidMainCoefficientInput& in,NativeSolidMainCoefficientResult* out) {
  using namespace coefficient_detail;
  if(!out||!Nonnegative(in.second_fill)||!Nonnegative(in.second_bulk)||
      !Finite(in.second_volume)||in.second_volume==0.)return CoefficientStatus::InvalidInput;
  if(in.first.layout!=SolidLayout::EightSlot)return CoefficientStatus::UnsupportedProfile;
  NativeSolidMainCoefficientResult first;
  const auto status=SolidMainContribution(in.first,&first,VolumeDomain::SignedNonzero,MainFaceKind::Internal);
  if(status!=CoefficientStatus::Ok)return status;
  const auto& a=in.first;
  const double second=a.scale*in.second_fill*a.area*a.area*in.second_bulk/in.second_volume;
  NativeSolidMainCoefficientResult result;
  // Literal native association and final internal-face sign, before INCOQ3
  // may replace this contribution with an incident shell's coefficient.
  result.stiffness=-(.5*(second+first.stiffness));
  result.characteristic_length=.5*(first.characteristic_length+in.second_volume/a.area);
  if(!Finite(result.stiffness)||!Finite(result.characteristic_length))
    return CoefficientStatus::NonfiniteResult;
  *out=result;return CoefficientStatus::Ok;
}
} // namespace tlfea::contact::radioss_type25
