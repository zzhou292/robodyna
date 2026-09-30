// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Tile.h"
namespace tl::fea::beam18::batch_detail::measurement {
TL_BEAM18_HD inline void Read(Tile& tile,unsigned lane,const Storage& state,unsigned trial,
    std::size_t p,double time,std::uint64_t epoch) noexcept {
  tile.status[lane]=state.status[p];
  // Failed evaluation leaves no readable candidate payload.
  if(tile.status[lane])return;
  const auto& parent=state.parents[p];const auto& now=state.slab[trial][p];
  tile.valid[lane]=ValidResult(parent,state.materials[parent.material_index],now,time,epoch);
  if(tile.valid[lane]!=1)return;
  for(unsigned c=0;c<2;++c)tile.work[lane][c]=now.diagnostics.internal_work_increment_j[c];
  tile.plastic[lane]=now.diagnostics.plastic_work_increment_j;
  tile.native_dt[lane]=now.diagnostics.minimum_unscaled_dt_s;
}
TL_BEAM18_HD inline MeasurementOperands Load(const Tile& tile,unsigned lane) noexcept {
  return {{tile.work[lane][0],tile.work[lane][1]},tile.plastic[lane],tile.native_dt[lane]};
}
}
