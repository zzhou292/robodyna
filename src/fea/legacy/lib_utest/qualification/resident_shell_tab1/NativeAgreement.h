#pragma once
#include "Values.h"
#include "../shell_placement_force/native/NativePacket.h"
#include "../shell_tab1_force/NativeAgreement.h"

namespace resident_tab1_test {
namespace native = placed::native;
// Compare only actual resident observations, without inventing transient point
// increments that the resident readback does not expose.
template<class F, class Force, class Interval>
void NativeAgreement(const Force& force, const fe::ShellBatchSectionState& section,
    const fe::ShellBatchFailureState& sidecar, const native::Packet& n,
    const Interval& interval, double old_thickness, double old_plastic_work) {
  using failure_force_test::Append;
  using failure_force_test::ArrayAgreement;
  constexpr bool quad = std::is_same_v<F, placed::Q>;
  failure_force_test::native::Packet geometry;
  geometry.force = n.force;
  geometry.planar = n.planar;
  failure_force_test::Geometry(force, geometry, interval);
  ASSERT_TRUE(force.proposed_history.prepared());
  EXPECT_EQ(force.proposed_history.stamp().time, interval.base_time + interval.dt);
  EXPECT_EQ(force.proposed_history.stamp().sample_index, interval.sample_index);
  ArrayAgreement(tab1_force_test::native::History(force.proposed_history.data()), n.history.data());
  std::vector<double> loads;
  Append(loads, force.internal_force);
  Append(loads, force.internal_couple);
  ArrayAgreement(loads, n.force.data() + (quad ? 118 : 64));
  ArrayAgreement(failure_force_test::Diagnostics(force.diagnostics), n.force.data() + (quad ? 142 : 82));
  EXPECT_EQ(sidecar.policy(), fe::ShellFailurePolicy::Tab1AnyPoint);
  ASSERT_NE(sidecar.tab1_points(), nullptr);
  EXPECT_EQ(sidecar.active, n.history[quad ? 37 : 25] == 1.);
  for (unsigned p = 0; p < 3; ++p) {
    for (unsigned c = 0; c < 5; ++c) {
      tab1_test::Close(section.history.point[p].stress[c], n.points[7 * p + c], 1e-8);
      tab1_test::Close(sidecar.current_force_point[p].stress[c], n.point_values[13 * p + c], 1e-8);
    }
    tab1_test::Close(section.history.point[p].plastic_strain, n.points[7 * p + 5]);
    tab1_test::Close(section.history.point[p].filtered_rate_per_s, n.points[7 * p + 6], 1e-10);
    const auto& point = sidecar.tab1_points()[p];
    tab1_test::Close(point.damage, n.failures[5 * p]);
    EXPECT_EQ(point.failure_time_s, n.failures[5 * p + 1]);
    EXPECT_EQ(point.point_active, n.failures[5 * p + 2] == 1.);
    tab1_test::Close(point.maximum_damage, n.failures[5 * p + 3]);
    EXPECT_EQ(point.table_segment, n.failures[5 * p + 4]);
  }
  const auto& d = section.diagnostics;
  tab1_test::Close(d.plastic_work_density_increment * old_thickness * force.kinematics.area, n.diagnostics[0], 1e-12);
  tab1_test::Close(d.mean_plastic_strain, n.diagnostics[1]);
  tab1_test::Close(d.maximum_plastic_strain, n.diagnostics[2]);
  tab1_test::Close(d.mean_tangent_ratio, n.diagnostics[3]);
  tab1_test::Close(d.minimum_tangent_ratio, n.diagnostics[4]);
  tab1_test::Close(d.mean_yield_before_pa, n.diagnostics[5], 1e-8);
  tab1_test::Close(d.last_point_yield_before_pa, n.diagnostics[6], 1e-8);
  tab1_test::Close(section.cumulative_plastic_work_J, old_plastic_work + n.diagnostics[0], 1e-12);
}
} // namespace resident_tab1_test
