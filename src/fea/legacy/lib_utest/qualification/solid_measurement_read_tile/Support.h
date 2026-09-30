// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/solids/resident/Storage.h"
#include "lib_src/elements/solids/resident/measurement/Read.h"
#include "ReferenceFinalize.h"
#include <gtest/gtest.h>
#include <vector>
#include <cmath>
#include <limits>
namespace solid_read_tile_test {
namespace s=tl::fea::solids;namespace b=s::batch_detail;namespace tile=b::measurement;
namespace fe=tl::fea;
inline s::BatchDiagnostics Seed(bool valid,double incoming) {
  s::BatchDiagnostics out;out.source_instance_id=61;out.owner_id=67;out.configuration_id=71;out.qualification_id=73;
  out.base_epoch=79;out.attempt=83;out.base_time=-0.;out.velocity_time=.125;
  out.base_velocity_time=-.125;out.kick_dt=-.25;out.phase=s::BatchPhase::Prepared;
  out.valid=valid;out.has_completed_interval=true;out.accepted_force_assembled=true;
  for(unsigned i=0;i<5;++i) {
    out.parent_count[i]=91+i;out.native_internal_work_increment_j[i]=incoming;
    out.physical_hourglass_work_increment_j[i]=(i%2)?-.5:.25;
  }
  out.plastic_work_increment_j=3;out.internal_kick_work_j=incoming;
  out.internal_drift_work_j=-incoming;out.minimum_native_dt_s=-17;return out;
}
inline b::Control Poison() {
  b::Control out{};out.status=s::BatchStatus::NonfiniteResult;out.family=s::Family::Solid18Law90;
  out.parent=101;out.node=103;out.element_status=107;out.diagnostics=Seed(true,109);return out;
}
inline void Same(const b::Control& a,const b::Control& c) {
  EXPECT_EQ(a.status,c.status);EXPECT_EQ(a.family,c.family);EXPECT_EQ(a.parent,c.parent);
  EXPECT_EQ(a.node,c.node);EXPECT_EQ(a.element_status,c.element_status);
  EXPECT_TRUE(b::SameDiagnostics(a.diagnostics,c.diagnostics));
}
template<class Traits> struct Rows {
  std::vector<int> status;
  std::vector<std::uint8_t> valid;
  std::vector<b::MeasurementOperands<Traits::nodes>> values;
  Rows(b::Storage& state,std::size_t count):status(count,0),valid(count,1),values(count) {
    auto& family=b::FamilyStorage<Traits>(state);family.count=count;
    family.status=status.data();family.result_valid=valid.data();family.measurement=values.data();
    for(std::size_t p=0;p<count;++p) {
      auto& row=values[p];row.work=p%3?-.25:.5;row.hourglass_work=.125;row.plastic_work=.25;
      row.native_dt=.01/(1+p);
      for(unsigned n=0;n<Traits::nodes;++n) {row.kick[n]=.25*(n+1);row.drift[n]=-.125*(n+1);}
    }
  }
};
} // namespace solid_read_tile_test
