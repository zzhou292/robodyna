// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"

namespace solid_operands_test {
template<class Traits> void SeedForces(d::Storage& state) {
  auto& family = d::FamilyStorage<Traits>(state);
  const double force[]{1e16, 1, -1e16, -0.0, DBL_MIN, -DBL_MIN, .5, -.5};
  for (unsigned n = 0; n < Traits::nodes; ++n) {
    family.slab[0][0].cache.rhs_force_n[n] = {force[n], -force[n], force[n]};
    family.slab[1][0].cache.rhs_force_n[n] = {11, 13, 17};
  }
}
TEST(SolidMeasurementOperands, FiveFamiliesRetainAcceptedRhsSlotOrderAndSeededRounding) {
  HostRig rig;
  ASSERT_TRUE(rig.Initialize());
  const auto count = rig.config.owner.node_count;
  std::vector<double> x(3 * count), base(x), velocity(x), base_velocity(x);
  for (std::size_t n = 0; n < x.size(); ++n) {
    base[n] = .03125 * double(n);
    x[n] = base[n] + (n % 2 ? -.125 : .125);
    velocity[n] = n % 2 ? -3 : 3;
    base_velocity[n] = n % 3 ? -0.0 : 1;
  }
  SeedForces<d::Traits18>(rig.state);
  SeedForces<d::Traits24>(rig.state);
  SeedForces<d::Traits6z>(rig.state);
  SeedForces<d::Traits18Law44>(rig.state);
  SeedForces<d::Traits18Law90>(rig.state);
  fe::NodalPreparedView view;
  view.base_kinematics.position_xyz = base.data();
  view.base_kinematics.velocity_xyz = base_velocity.data();
  view.kinematics.position_xyz = x.data();
  view.kinematics.velocity_xyz = velocity.data();
  for (const double dt : {.125, 0.0, -0.0, -.125}) {
    view.kick_dt = dt;
    CompareAll(rig, &view, 7);
    EXPECT_EQ(rig.state.control.status, s::BatchStatus::Success);
  }
}

template<class Traits> void InitialNoAccepted(HostRig& rig) {
  auto& family = d::FamilyStorage<Traits>(rig.state);
  auto* saved = family.slab[0];
  family.slab[0] = nullptr;
  Stage<Traits>(rig.state, 0, 1, nullptr);
  EXPECT_EQ(family.result_valid[0], 1);
  for (unsigned n = 0; n < Traits::nodes; ++n) {
    EXPECT_EQ(family.measurement[0].kick[n], 0);
    EXPECT_EQ(family.measurement[0].drift[n], 0);
  }
  family.slab[0] = saved;
}
TEST(SolidMeasurementOperands, InitialNullViewNeverReadsAcceptedAndAllResolvedMaterialsWork) {
  for (int flag : {1, 2}) for (bool analytic : {false, true}) {
    HostRig rig;
    ASSERT_TRUE(rig.Initialize(flag, analytic));
    InitialNoAccepted<d::Traits18>(rig);
    InitialNoAccepted<d::Traits24>(rig);
    InitialNoAccepted<d::Traits6z>(rig);
    InitialNoAccepted<d::Traits18Law44>(rig);
    InitialNoAccepted<d::Traits18Law90>(rig);
    CompareAll(rig);
    EXPECT_EQ(rig.state.control.status, s::BatchStatus::Success);
  }
}

template<class Traits> void Unavailable(HostRig& rig) {
  auto& family = d::FamilyStorage<Traits>(rig.state);
  const auto original = family;
  family.status[0] = 9;
  family.parents = nullptr;
  family.slab[0] = family.slab[1] = nullptr;
  std::memset(family.measurement, 0xff, sizeof(*family.measurement));
  Stage<Traits>(rig.state, 0, 1, reinterpret_cast<const fe::NodalPreparedView*>(1));
  EXPECT_EQ(family.result_valid[0], 0);
  const d::MeasurementOperands<Traits::nodes> empty;
  EXPECT_EQ(std::memcmp(family.measurement, &empty, sizeof(empty)), 0);
  family = original;
  family.status[0] = 0;
  family.slab[1][0].cache.rhs_force_n[Traits::nodes - 1].z = NAN;
  family.slab[0] = nullptr;
  Stage<Traits>(rig.state, 0, 1, reinterpret_cast<const fe::NodalPreparedView*>(1));
  EXPECT_EQ(family.result_valid[0], 0);
  family = original;
  family.slab[1][0] = family.slab[0][0];
  Stage<Traits>(rig.state, 0, 1, nullptr);
  EXPECT_EQ(family.result_valid[0], 1);
}
TEST(SolidMeasurementOperands, BadStatusAndBadResultSkipUnavailableInputsThenRetryFreshly) {
  HostRig rig;
  ASSERT_TRUE(rig.Initialize());
  Unavailable<d::Traits18>(rig);
  Unavailable<d::Traits24>(rig);
  Unavailable<d::Traits6z>(rig);
  Unavailable<d::Traits18Law44>(rig);
  Unavailable<d::Traits18Law90>(rig);
  CompareAll(rig);
}

TEST(SolidMeasurementOperands, EarlierFoldOverflowWinsOverLaterInvalidHistoryAndStatus) {
  HostRig rig;
  ASSERT_TRUE(rig.Initialize());
  MeasuredRows<d::Traits18> rows(rig.state, 3);
  rows.trial[0].cache.diagnostics.internal_work_increment_j = DBL_MAX;
  rows.trial[1].cache.diagnostics.internal_work_increment_j = DBL_MAX;
  for (int status : {0, 9}) {
    rows.status[2] = status;
    rows.trial[2].cache.rhs_force_n[7].z = NAN;
    CompareAll(rig);
    EXPECT_EQ(rig.state.control.status, s::BatchStatus::NonfiniteResult);
    EXPECT_EQ(rig.state.control.parent, 1u);
    EXPECT_EQ(rig.state.control.element_status, 0);
  }
  rows.trial[1].cache.diagnostics.internal_work_increment_j = -DBL_MAX;
  rows.status[2] = 9;
  CompareAll(rig);
  EXPECT_EQ(rig.state.control.status, s::BatchStatus::ElementFailure);
  EXPECT_EQ(rig.state.control.parent, 2u);
}
TEST(SolidMeasurementOperands, CrossFamilyOverflowKeepsLiteralPartialDiagnostics) {
  HostRig rig;
  ASSERT_TRUE(rig.Initialize());
  rig.state.solid18.slab[1][0].cache.diagnostics.plastic_work_increment_j = DBL_MAX;
  rig.state.solid18_law44.slab[1][0].cache.diagnostics.plastic_work_increment_j = DBL_MAX;
  rig.state.solid18_law90.slab[1][0].cache.rhs_force_n[7].z = NAN;
  CompareAll(rig);
  EXPECT_EQ(rig.state.control.family, s::Family::Solid18Law44);
  EXPECT_EQ(rig.state.control.diagnostics.parent_count[4], 0u);
  rig.state.solid18_law44.slab[1][0].cache.diagnostics.plastic_work_increment_j = 0;
  CompareAll(rig);
  EXPECT_EQ(rig.state.control.family, s::Family::Solid18Law90);
  rig.state.solid18_law90.slab[1][0] = rig.state.solid18_law90.slab[0][0];
  CompareAll(rig);
  EXPECT_EQ(rig.state.control.status, s::BatchStatus::Success);
}
TEST(SolidMeasurementOperands, DerivedOverflowIsFoldedBeforeItsOriginalParentFiniteCheck) {
  HostRig rig;
  ASSERT_TRUE(rig.Initialize());
  const auto nodes = rig.config.owner.node_count;
  std::vector<double> positions(3 * nodes), velocity(3 * nodes, DBL_MAX);
  fe::NodalPreparedView view;
  view.base_kinematics.position_xyz = view.kinematics.position_xyz = positions.data();
  view.base_kinematics.velocity_xyz = view.kinematics.velocity_xyz = velocity.data();
  view.kick_dt = 1;
  rig.state.solid18.slab[0][0].cache.rhs_force_n[0] = {1, 2, 3};
  rig.state.solid18.slab[1][0].cache.diagnostics.internal_work_increment_j = 19;
  CompareAll(rig, &view);
  EXPECT_EQ(rig.state.solid18.result_valid[0], 1);
  EXPECT_EQ(rig.state.control.parent, 0u);
  EXPECT_EQ(rig.state.control.diagnostics.native_internal_work_increment_j[0], 19);
  EXPECT_EQ(rig.state.control.status, s::BatchStatus::NonfiniteResult);
}
} // namespace solid_operands_test
