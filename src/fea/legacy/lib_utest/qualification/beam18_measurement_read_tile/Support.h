// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/beam18/resident/measurement/Read.h"
#include "lib_utest/qualification/beam18_force/TestSupport.h"
#include "ReferenceMeasure.h"
#include <gtest/gtest.h>
#include <cstring>
namespace beam18_read_tile_test {
namespace fe=tl::fea;namespace b=fe::beam18;namespace d=b::batch_detail;namespace tile=d::measurement;
inline std::uint64_t Bits(double x){std::uint64_t n;std::memcpy(&n,&x,8);return n;}
inline b::BatchDiagnostics Seed(bool valid=false,double value=0.) {
  b::BatchDiagnostics x;x.source_instance_id=11;x.owner_id=13;x.configuration_id=17;x.qualification_id=19;
  x.attempt=7;x.base_epoch=0;x.epoch=0;x.time=0;x.base_time=-.5;x.velocity_time=-.25;x.base_velocity_time=-.75;x.kick_dt=.5;
  x.phase=b::BatchPhase::Prepared;x.valid=valid;x.has_completed_interval=true;x.accepted_force_assembled=true;x.parent_count=999;
  x.native_internal_work_increment_j[0]=x.native_internal_work_increment_j[1]=value;
  x.internal_kick_work_j=x.internal_drift_work_j=value;x.plastic_work_increment_j=-0.;x.minimum_native_dt_s=-19;return x;
}
inline d::Control Poison(){d::Control c;c.status=b::BatchStatus::Unusable;c.parent=31;c.node=29;c.element_status=5;c.diagnostics=Seed(true,-17);return c;}
inline void Same(const d::Control& a,const d::Control& z) {
  EXPECT_EQ(a.status,z.status);EXPECT_EQ(a.parent,z.parent);EXPECT_EQ(a.node,z.node);EXPECT_EQ(a.element_status,z.element_status);
  const auto&x=a.diagnostics;const auto&y=z.diagnostics;
#define FIELD(f) EXPECT_EQ(x.f,y.f)<<#f
  FIELD(source_instance_id);FIELD(owner_id);FIELD(configuration_id);FIELD(qualification_id);FIELD(epoch);FIELD(base_epoch);FIELD(attempt);
  FIELD(phase);FIELD(valid);FIELD(has_completed_interval);FIELD(accepted_force_assembled);FIELD(parent_count);
#undef FIELD
#define REAL(f) EXPECT_EQ(Bits(x.f),Bits(y.f))<<#f
  REAL(time);REAL(base_time);REAL(velocity_time);REAL(base_velocity_time);REAL(kick_dt);REAL(plastic_work_increment_j);REAL(internal_kick_work_j);REAL(internal_drift_work_j);REAL(minimum_native_dt_s);
#undef REAL
  for(unsigned c=0;c<2;++c)EXPECT_EQ(Bits(x.native_internal_work_increment_j[c]),Bits(y.native_internal_work_increment_j[c]));
}
}
