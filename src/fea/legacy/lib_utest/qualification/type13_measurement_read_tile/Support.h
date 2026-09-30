// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/type13/resident/measurement/Read.h"
#include "ReferenceMeasurement.h"
#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>
namespace type13_read_tile_test {
namespace t=tl::fea::type13;namespace b=t::batch_detail;namespace tile=b::measurement;
inline std::uint64_t Bits(double v){std::uint64_t x;std::memcpy(&x,&v,8);return x;}
inline t::BatchDiagnostics Seed(bool valid=false,double value=.25) {
  t::BatchDiagnostics d;d.source_instance_id=11;d.owner_id=13;d.configuration_id=17;d.qualification_id=19;
  d.epoch=5;d.base_epoch=4;d.attempt=7;d.time=.125;d.base_time=.0625;
  d.velocity_time=.09375;d.base_velocity_time=.03125;d.kick_dt=.0625;
  d.phase=t::BatchPhase::Prepared;d.valid=valid;d.has_completed_interval=true;d.accepted_force_assembled=true;
  d.element_count=101;d.active_count=103;d.newly_failed_count=107;
  for(unsigned c=0;c<t::ChannelCount;++c)d.internal_work_J[c]=d.internal_work_increment_J[c]=value;
  d.internal_kick_work_J=d.internal_drift_work_J=value;d.minimum_native_dt_s=-19;return d;
}
inline b::Measurement Row(std::size_t p) {
  b::Measurement row;const double v[]={0x1p54,1,-0x1p54,-0.,0x1p-1022,-0x1p-1022,.25,-.25};
  for(unsigned c=0;c<t::ChannelCount;++c){row.work[c]=v[(p+c)%8];row.increment[c]=v[(p+2*c)%8];}
  for(unsigned n=0;n<2;++n){row.kick[n]=v[(p+n)%8];row.drift[n]=v[(p+2+n)%8];}
  row.native_dt=.01+double(p%3)*.001;row.active=p%3!=0;row.newly_failed=p%3==0;return row;
}
inline b::Control Poison(){b::Control c;c.status=t::BatchStatus::Unusable;c.element_status=t::Status::DegenerateGeometry;
  c.element=31;c.node=29;c.diagnostics=Seed(true,-17);return c;}
inline void Same(const b::Control& x,const b::Control& y) {
  EXPECT_EQ(x.status,y.status);EXPECT_EQ(x.element_status,y.element_status);EXPECT_EQ(x.element,y.element);EXPECT_EQ(x.node,y.node);
  const auto&a=x.diagnostics;const auto&b=y.diagnostics;
#define FIELD(f) EXPECT_EQ(a.f,b.f)<<#f
  FIELD(source_instance_id);FIELD(owner_id);FIELD(configuration_id);FIELD(qualification_id);FIELD(epoch);FIELD(base_epoch);FIELD(attempt);
  FIELD(phase);FIELD(valid);FIELD(has_completed_interval);FIELD(accepted_force_assembled);FIELD(element_count);FIELD(active_count);FIELD(newly_failed_count);
#undef FIELD
#define REAL(f) EXPECT_EQ(Bits(a.f),Bits(b.f))<<#f
  REAL(time);REAL(base_time);REAL(velocity_time);REAL(base_velocity_time);REAL(kick_dt);REAL(internal_kick_work_J);REAL(internal_drift_work_J);REAL(minimum_native_dt_s);
#undef REAL
  for(unsigned c=0;c<t::ChannelCount;++c){EXPECT_EQ(Bits(a.internal_work_J[c]),Bits(b.internal_work_J[c]));EXPECT_EQ(Bits(a.internal_work_increment_J[c]),Bits(b.internal_work_increment_J[c]));}
}
}
