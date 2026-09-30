// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Channels.h"
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
    const unsigned count=remaining<Threads?unsigned(remaining):Threads;
    if(lane<count)Read<Traits>(tile,lane,family,first+lane,view!=nullptr);
    const bool invalid=lane<count && (tile.status[lane]!=0 || tile.valid[lane]!=1);
    const unsigned mask=__ballot_sync(0xffffffffu,invalid);
    if((lane&31)==0)tile.invalid[lane/32]=mask;
    if(!lane)SeedChannels(tile,control.diagnostics,family_index);
    __syncthreads();
    const bool serial=tile.invalid[0] || tile.invalid[1];
    if(!serial) {
      if(lane<4)ScalarChannel(tile,lane,count);
      if(lane==4)MinimumChannel(tile,count);
      if(lane==32 || lane==33)WorkChannel<Traits::nodes>(tile,lane-28,count,view!=nullptr);
    }
    __syncthreads();
    if(!lane) {
      // Speculative channels never touch Control. Replay recovers every field
      // through the exact first failure, including all failing-parent writes.
      if(serial || !ChannelsFinite(tile))
        tile.proceed=ReplayTile<Traits>(control,family_index,first,count,view,tile);
      else StoreChannels(tile,control.diagnostics,family_index);
    }
    __syncthreads();
    if(!tile.proceed)break;
  }
  return tile.proceed;
}
} // namespace tl::fea::solids::batch_detail::measurement
