// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Binding.h"
#include "../../HistoryPhase.h"
namespace tlfea::contact::radioss_type25::selection::lifecycle {
namespace detail {
TL_MATH_HOST_DEVICE inline Status Keep(const SourceView& source,Occurrence& occurrence,
    RowResult& state) {
  if(occurrence.secondary<=0)return Status::Ok;
  if(!occurrence.cache_initialized)return Status::UndefinedNativeInput;
  std::int64_t global=state.history.row.irtlm[0];if(global<0)global=-global;
  bool keep=false;
  if(global==source.mains[occurrence.local_main-1].global_id) {
    for(const auto& sector:occurrence.cache.sector)
      if(!(sector.defined&PenetrationDefined))return Status::UndefinedNativeInput;
    const auto& s=occurrence.cache.sector;
    const double sum=((s[0].penetration+s[1].penetration)+s[2].penetration)+s[3].penetration;
    if(sum!=0)keep=true;
    else {
      ++state.zero_sum_resets;
      for(auto& marker:state.history.row.irtlm)marker=0;
    }
  }
  if(!keep)occurrence.secondary=-occurrence.secondary;
  else ++state.kept_count;
  return Status::Ok;
}
} // namespace detail
// Exact MAINF post-force transition. Call only AFTER initial-offset/history,
// normal/friction and force work, and BEFORE physical owner publication.
TL_MATH_HOST_DEVICE inline Status FinishNativeRow(const NativeGeometryHistory& input,
    NativeGeometryHistory* output) {
  if(!output||!geometry_detail::ValidRow(input.row)||input.row.irtlm[0]==INT_MIN)
    return Status::InvalidInput;
  auto staged=input;
  if(staged.row.irtlm[0]<0)staged.row.irtlm[0]=-staged.row.irtlm[0];
  *output=staged;return Status::Ok;
}
} // namespace tlfea::contact::radioss_type25::selection::lifecycle
