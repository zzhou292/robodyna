// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Tile.h"
namespace tl::fea::type13::batch_detail::measurement {
TL_TYPE13_HD inline void Read(Tile& tile,unsigned lane,const Measurement& row) noexcept {
  for(unsigned c=0;c<ChannelCount;++c) {tile.work[lane][c]=row.work[c];tile.increment[lane][c]=row.increment[c];}
  for(unsigned n=0;n<2;++n) {tile.kick[lane][n]=row.kick[n];tile.drift[lane][n]=row.drift[n];}
  tile.native_dt[lane]=row.native_dt;tile.active[lane]=row.active;tile.newly_failed[lane]=row.newly_failed;
}
TL_TYPE13_HD inline Measurement Load(const Tile& tile,unsigned lane) noexcept {
  Measurement row;
  for(unsigned c=0;c<ChannelCount;++c) {row.work[c]=tile.work[lane][c];row.increment[c]=tile.increment[lane][c];}
  for(unsigned n=0;n<2;++n) {row.kick[n]=tile.kick[lane][n];row.drift[n]=tile.drift[lane][n];}
  row.native_dt=tile.native_dt[lane];row.active=tile.active[lane];row.newly_failed=tile.newly_failed[lane];return row;
}
}
