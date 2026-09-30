// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "UnitConversions.h"
namespace tlfea::contact::radioss_type25 {
TL_MATH_HOST_DEVICE inline NormalStatus EvaluateSiNormal(const ResolvedNormalConfig& config,
    UnitScale units, const SiNormalInput& input, const SiNormalHistory& history,
    SiNormalResult* output) {
  units_detail::Factors f;
  if (!output || !normal_detail::Valid(input) || !normal_detail::Valid(history) ||
      !units_detail::Make(units, f)) return NormalStatus::InvalidInput;
  const auto in = units_detail::ToNative(input, f);
  const auto old = units_detail::ToNative(history, f);
  if (!normal_detail::Valid(in) || !normal_detail::Valid(old)) return NormalStatus::NonfiniteResult;
  NativeNormalResult native;
  const auto status = EvaluateNativeNormal(config, in, old, &native);
  if (status != NormalStatus::Ok) return status;
  const auto result = units_detail::ToSi(native, f);
  if (!normal_detail::Finite(result)) return NormalStatus::NonfiniteResult;
  *output = result;
  return NormalStatus::Ok;
}
} // namespace tlfea::contact::radioss_type25
