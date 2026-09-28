// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Read.h"
namespace tl::fea::solids::batch_detail::measurement {
// Each scalar keeps its incoming value and its complete parent/local sequence.
// No tile subtotal is formed. The serial parent fold remains the error authority.
TL_BRICK_HD inline void SeedChannels(Tile& t,const BatchDiagnostics& d,unsigned family) noexcept {
  t.sum[0]=d.native_internal_work_increment_j[family];
  t.sum[1]=d.physical_hourglass_work_increment_j[family];
  t.sum[2]=d.distortion_work_increment_j[family];
  t.sum[3]=d.plastic_work_increment_j;
  t.sum[4]=d.internal_kick_work_j;t.sum[5]=d.internal_drift_work_j;
  t.minimum_dt=d.minimum_native_dt_s;
}
TL_BRICK_HD inline void ScalarChannel(Tile& t,unsigned channel,unsigned count) noexcept {
  const double* rows=channel==0?t.work:channel==1?t.hourglass_work:
      channel==2?t.distortion_work:t.plastic_work;
  double value=t.sum[channel];bool finite=true;
  for(unsigned p=0;p<count;++p) {
    value+=rows[p];
    finite=tl::math::Finite(value)&&finite;
  }
  t.sum[channel]=value;t.finite[channel]=finite;
}
template<unsigned Slots>
TL_BRICK_HD inline void WorkChannel(Tile& t,unsigned channel,unsigned count,bool work) noexcept {
  const auto* rows=channel==4?t.kick:t.drift;
  double value=t.sum[channel];bool finite=true;
  for(unsigned p=0;p<count;++p) {
    if(work) for(unsigned slot=0;slot<Slots;++slot) value+=rows[p][slot];
    // Original finiteness is observed after all slots of this parent.
    finite=tl::math::Finite(value)&&finite;
  }
  t.sum[channel]=value;t.finite[channel]=finite;
}
TL_BRICK_HD inline void MinimumChannel(Tile& t,unsigned count) noexcept {
  double value=t.minimum_dt;
  for(unsigned p=0;p<count;++p) if(t.native_dt[p]<value)value=t.native_dt[p];
  t.minimum_dt=value;
}
TL_BRICK_HD inline bool ChannelsFinite(const Tile& t) noexcept {
  for(bool finite:t.finite)if(!finite)return false;
  return true;
}
TL_BRICK_HD inline void StoreChannels(const Tile& t,BatchDiagnostics& d,unsigned family) noexcept {
  d.native_internal_work_increment_j[family]=t.sum[0];
  d.physical_hourglass_work_increment_j[family]=t.sum[1];
  d.distortion_work_increment_j[family]=t.sum[2];d.plastic_work_increment_j=t.sum[3];
  d.internal_kick_work_j=t.sum[4];d.internal_drift_work_j=t.sum[5];
  d.minimum_native_dt_s=t.minimum_dt;
}
template<class Traits>
TL_BRICK_HD inline bool ReplayTile(Control& control,unsigned family,std::size_t first,
    unsigned count,const NodalPreparedView* view,const Tile& tile) noexcept {
  for(unsigned local=0;local<count;++local) {
    if(tile.status[local]!=0 || tile.valid[local]!=1) {
      control.status=tile.status[local]?BatchStatus::ElementFailure:BatchStatus::NonfiniteResult;
      control.family=Traits::family;control.parent=first+local;control.element_status=tile.status[local];
      return false;
    }
    const auto value=Load<Traits>(tile,local,view!=nullptr);
    if(!AccumulateMeasurementOperand<Traits>(control,family,first+local,value,view))return false;
  }
  return true;
}
} // namespace tl::fea::solids::batch_detail::measurement
