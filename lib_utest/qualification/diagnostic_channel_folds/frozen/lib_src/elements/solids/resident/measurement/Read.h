// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Tile.h"
#include "../MeasurementValues.h"
namespace tl::fea::solids::batch_detail::measurement {
template<class Traits>
TL_BRICK_HD inline void Read(Tile& tile,unsigned lane,const DeviceFamily<Traits>& family,
    std::size_t parent,bool work) noexcept {
  tile.status[lane]=family.status[parent];
  // Keep the original short circuit: failed rows need no validity or payload.
  if(tile.status[lane]) return;
  tile.valid[lane]=family.result_valid[parent];
  if(tile.valid[lane]!=1) return;
  const auto& value=family.measurement[parent];
  tile.work[lane]=value.work;tile.hourglass_work[lane]=value.hourglass_work;tile.distortion_work[lane]=value.distortion_work;
  tile.plastic_work[lane]=value.plastic_work;tile.native_dt[lane]=value.native_dt;
  if(work) for(unsigned local=0;local<Traits::nodes;++local) {
    tile.kick[lane][local]=value.kick[local];tile.drift[lane][local]=value.drift[local];
  }
}
template<class Traits>
TL_BRICK_HD inline MeasurementOperands<Traits::nodes> Load(const Tile& tile,
    unsigned lane,bool work) noexcept {
  MeasurementOperands<Traits::nodes> value;
  value.work=tile.work[lane];value.hourglass_work=tile.hourglass_work[lane];value.distortion_work=tile.distortion_work[lane];
  value.plastic_work=tile.plastic_work[lane];value.native_dt=tile.native_dt[lane];
  if(work) for(unsigned local=0;local<Traits::nodes;++local) {
    value.kick[local]=tile.kick[lane][local];value.drift[local]=tile.drift[lane][local];
  }
  return value;
}
} // namespace tl::fea::solids::batch_detail::measurement
