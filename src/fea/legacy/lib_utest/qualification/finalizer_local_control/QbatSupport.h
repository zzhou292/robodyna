// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_utest/qualification/qbat_measurement_operands/Fixture.h"
#include "Control2733Qbat.h"
namespace final_control_test::qbat {
namespace f=qbat_measurement_test;
namespace q=tl::fea::qbat;
namespace b=q::batch_detail;
namespace m=q::mapped;
inline q::BatchDiagnostics Seed(bool valid,double incoming) {
  auto out=f::g::Identity(2);
  out.owner_id=71;out.configuration_id=73;out.qualification_id=79;
  out.base_epoch=83;out.attempt=89;out.base_time=-0.;out.base_velocity_time=-.25;
  out.velocity_time=.125;out.kick_dt=-.125;out.phase=q::BatchPhase::Prepared;
  out.valid=valid;out.has_completed_interval=true;out.accepted_force_assembled=true;
  out.element_count=91;out.active_count=7;out.newly_removed_count=11;
  out.internal_work_j[0]=incoming;out.internal_work_j[1]=-0.;
  out.internal_work_increment_j[0]=.25;out.internal_work_increment_j[1]=-.5;
  out.plastic_work_j=3;out.plastic_work_increment_j=-4;
  out.numerical_viscous_work_j=5;out.numerical_viscous_work_increment_j=-6;
  out.minimum_area_ratio=-7;out.minimum_thickness_ratio=-8;out.minimum_native_dt=-9;
  out.maximum_displacement=.01;out.maximum_absolute_strain=.02;
  out.internal_kick_work=incoming;out.internal_drift_work=-incoming;
  return out;
}
inline b::Control Poison() {
  b::Control out{};out.status=q::BatchStatus::NonfiniteResult;
  out.element_status=q::Status::kInvalidReference;out.element=17;out.node=19;
  out.diagnostics=Seed(true,123);return out;
}
inline void Same(const b::Control& actual,const b::Control& expected) {
  EXPECT_EQ(actual.status,expected.status);EXPECT_EQ(actual.element_status,expected.element_status);
  EXPECT_EQ(actual.element,expected.element);EXPECT_EQ(actual.node,expected.node);
  EXPECT_TRUE(b::SameDiagnostics(actual.diagnostics,expected.diagnostics));
}
inline void Compare(f::Fixture& fixture,const q::BatchDiagnostics& identity,unsigned blocks=1) {
  const auto view=fixture.input.Prepared(2);const auto poison=Poison();
  fixture.host->control=poison;
  m::control2733::FinalizeMeasurement(*fixture.host,view,identity,blocks);
  const auto expected=fixture.host->control;
  fixture.host->control=poison;
  m::FinalizeMeasurement(*fixture.host,view,identity,blocks);
  Same(fixture.host->control,expected);
}
} // namespace final_control_test::qbat
