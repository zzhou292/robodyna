// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Tile.h"
#include "../MeasurementValues.h"
namespace tl::fea::qbat::mapped::measurement {
// Every row has a written valid byte. Failed preparation does not admit any
// other payload field, and inactive rows still contribute their real history.
TL_QBAT_HD inline void Read(Tile& tile,unsigned lane,const MeasurementParent& source) noexcept {
  tile.valid[lane]=source.valid;
  if(source.valid!=1) return;
  for(unsigned i=0;i<2;++i) tile.internal_work[lane][i]=source.internal_work[i];
  for(unsigned i=0;i<2;++i) tile.internal_increment[lane][i]=source.internal_increment[i];
  tile.plastic_work[lane]=source.plastic_work;
  tile.plastic_increment[lane]=source.plastic_increment;
  tile.viscous_work[lane]=source.viscous_work;
  tile.viscous_increment[lane]=source.viscous_increment;
  for(unsigned i=0;i<4;++i) tile.kick_operand[lane][i]=source.kick_operand[i];
  for(unsigned i=0;i<4;++i) tile.drift_operand[lane][i]=source.drift_operand[i];
  tile.area_ratio[lane]=source.area_ratio;
  tile.thickness_ratio[lane]=source.thickness_ratio;
  tile.native_dt[lane]=source.native_dt;
  tile.maximum_strain[lane]=source.maximum_strain;
  tile.active[lane]=source.active;
  tile.newly_removed[lane]=source.newly_removed;
}
TL_QBAT_HD inline MeasurementParent Load(const Tile& tile,unsigned lane) noexcept {
  MeasurementParent value;
  value.valid=tile.valid[lane];
  if(value.valid!=1) return value;
  for(unsigned i=0;i<2;++i) value.internal_work[i]=tile.internal_work[lane][i];
  for(unsigned i=0;i<2;++i) value.internal_increment[i]=tile.internal_increment[lane][i];
  value.plastic_work=tile.plastic_work[lane];
  value.plastic_increment=tile.plastic_increment[lane];
  value.viscous_work=tile.viscous_work[lane];
  value.viscous_increment=tile.viscous_increment[lane];
  for(unsigned i=0;i<4;++i) value.kick_operand[i]=tile.kick_operand[lane][i];
  for(unsigned i=0;i<4;++i) value.drift_operand[i]=tile.drift_operand[lane][i];
  value.area_ratio=tile.area_ratio[lane];
  value.thickness_ratio=tile.thickness_ratio[lane];
  value.native_dt=tile.native_dt[lane];
  value.maximum_strain=tile.maximum_strain[lane];
  value.active=tile.active[lane];
  value.newly_removed=tile.newly_removed[lane];
  return value;
}
} // namespace tl::fea::qbat::mapped::measurement
