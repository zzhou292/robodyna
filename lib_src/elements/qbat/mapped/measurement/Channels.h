// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Read.h"
namespace tl::fea::qbat::mapped::measurement {
TL_QBAT_HD inline void SeedChannels(Tile& t,const BatchDiagnostics& d) noexcept {
  for(unsigned c=0;c<2;++c) {t.sum[c]=d.internal_work_j[c];t.sum[2+c]=d.internal_work_increment_j[c];}
  t.sum[4]=d.plastic_work_j;t.sum[5]=d.plastic_work_increment_j;
  t.sum[6]=d.numerical_viscous_work_j;t.sum[7]=d.numerical_viscous_work_increment_j;
  t.sum[8]=d.internal_kick_work;t.sum[9]=d.internal_drift_work;
  t.minimum[0]=d.minimum_area_ratio;t.minimum[1]=d.minimum_thickness_ratio;t.minimum[2]=d.minimum_native_dt;
  t.maximum=d.maximum_absolute_strain;t.active_count=d.active_count;t.removed_count=d.newly_removed_count;
}
// Every channel is a separate original scalar += sequence. In particular,
// derived overflow is not rejected here: QBAT checks aggregates at the suffix.
TL_QBAT_HD inline void ScalarChannel(Tile& t,unsigned channel,unsigned count) noexcept {
  double value=t.sum[channel];
  if(channel<4) {
    const auto* rows=channel<2?t.internal_work:t.internal_increment;
    const unsigned c=channel%2;
    for(unsigned p=0;p<count;++p)value+=rows[p][c];
  } else {
    const double* rows=channel==4?t.plastic_work:channel==5?t.plastic_increment:
        channel==6?t.viscous_work:t.viscous_increment;
    for(unsigned p=0;p<count;++p)value+=rows[p];
  }
  t.sum[channel]=value;
}
TL_QBAT_HD inline void WorkChannel(Tile& t,unsigned channel,unsigned count,bool coupled) noexcept {
  if(!coupled)return;
  const auto* rows=channel==8?t.kick_operand:t.drift_operand;
  double value=t.sum[channel];
  for(unsigned p=0;p<count;++p)for(unsigned slot=0;slot<4;++slot)value-=rows[p][slot];
  t.sum[channel]=value;
}
TL_QBAT_HD inline void MinimumChannel(Tile& t,unsigned channel,std::size_t first,unsigned count) noexcept {
  const double* rows=channel==0?t.area_ratio:channel==1?t.thickness_ratio:t.native_dt;
  double value=t.minimum[channel];
  for(unsigned p=0;p<count;++p)if(!(first+p) || rows[p]<value)value=rows[p];
  t.minimum[channel]=value;
}
TL_QBAT_HD inline void MaximumChannel(Tile& t,unsigned count) noexcept {
  double value=t.maximum;
  for(unsigned p=0;p<count;++p)if(t.maximum_strain[p]>value)value=t.maximum_strain[p];
  t.maximum=value;
}
TL_QBAT_HD inline void CountChannel(Tile& t,bool removed,unsigned count) noexcept {
  const auto* rows=removed?t.newly_removed:t.active;
  auto value=removed?t.removed_count:t.active_count;
  for(unsigned p=0;p<count;++p)if(rows[p])++value;
  if(removed)t.removed_count=value;else t.active_count=value;
}
TL_QBAT_HD inline void StoreChannels(const Tile& t,BatchDiagnostics& d) noexcept {
  for(unsigned c=0;c<2;++c) {d.internal_work_j[c]=t.sum[c];d.internal_work_increment_j[c]=t.sum[2+c];}
  d.plastic_work_j=t.sum[4];d.plastic_work_increment_j=t.sum[5];
  d.numerical_viscous_work_j=t.sum[6];d.numerical_viscous_work_increment_j=t.sum[7];
  d.internal_kick_work=t.sum[8];d.internal_drift_work=t.sum[9];
  d.minimum_area_ratio=t.minimum[0];d.minimum_thickness_ratio=t.minimum[1];d.minimum_native_dt=t.minimum[2];
  d.maximum_absolute_strain=t.maximum;d.active_count=t.active_count;d.newly_removed_count=t.removed_count;
}
TL_QBAT_HD inline bool ReplayTile(const batch_detail::Model& model,const Tile& tile,
    std::size_t first,unsigned count,BatchDiagnostics& d) noexcept {
  for(unsigned local=0;local<count;++local) {
    if(tile.valid[local]!=1)return false;
    AccumulateMeasurementParent(model,Load(tile,local),first+local,d);
  }
  return true;
}
} // namespace tl::fea::qbat::mapped::measurement
