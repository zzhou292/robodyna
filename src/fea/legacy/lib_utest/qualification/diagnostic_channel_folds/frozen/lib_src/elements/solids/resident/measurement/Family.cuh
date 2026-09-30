// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Read.h"
namespace tl::fea::solids::batch_detail::measurement {
template<class Traits>
__device__ inline bool Measure(Storage& state,Control& control,unsigned family_index,
    const NodalPreparedView* view,Tile& tile) noexcept {
  auto& family=FamilyStorage<Traits>(state);
  const unsigned lane=threadIdx.x;
  if(!lane) {control.diagnostics.parent_count[family_index]=family.count;tile.proceed=true;}
  __syncthreads();
  for(std::size_t first=0;first<family.count;first+=Threads) {
    const auto remaining=family.count-first;
    const unsigned count=remaining<Threads ? unsigned(remaining) : Threads;
    if(lane<count) Read<Traits>(tile,lane,family,first+lane,view!=nullptr);
    __syncthreads();
    if(!lane) for(unsigned local=0;local<count;++local) {
      if(tile.status[local]!=0 || tile.valid[local]!=1) {
        control.status=tile.status[local] ? BatchStatus::ElementFailure : BatchStatus::NonfiniteResult;
        control.family=Traits::family;control.parent=first+local;control.element_status=tile.status[local];
        tile.proceed=false;break;
      }
      const auto value=Load<Traits>(tile,local,view!=nullptr);
      if(!AccumulateMeasurementOperand<Traits>(control,family_index,first+local,value,view)) {
        tile.proceed=false;break;
      }
    }
    __syncthreads();
    if(!tile.proceed) break;
  }
  return tile.proceed;
}
} // namespace tl::fea::solids::batch_detail::measurement
