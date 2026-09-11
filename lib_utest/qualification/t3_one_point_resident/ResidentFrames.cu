#include "ResidentFixture.h"

namespace t3_one_point_resident_test {
namespace {
std::array<double, 28> PointValues(const fe::ShellBatchOnePointSectionState& value) {
  std::array<double, 28> result{};
  unsigned i = 0;
  const auto& p = value.point;
  for (double s : p.current.history.stress) result[i++] = s;
  result[i++] = p.current.history.plastic_strain;
  result[i++] = p.current.history.filtered_rate_per_s;
  for (double d : {p.current.plastic_increment, p.current.tangent_ratio,
      p.current.elastic_thickness_strain, p.current.plastic_thickness_strain,
      p.current.yield_before_pa, p.current.equivalent_stress_pa, p.current.plastic_work_density}) result[i++] = d;
  for (double s : p.saved.stress) result[i++] = s;
  result[i++] = p.saved.plastic_strain;
  result[i++] = p.saved.filtered_rate_per_s;
  result[i++] = p.failure.history.damage;
  result[i++] = p.failure.history.failure_time_s;
  result[i++] = p.failure.history.point_active ? 1 : 0;
  result[i++] = p.failure.failed_now ? 1 : 0;
  result[i++] = p.reported_thickness_m;
  result[i++] = p.plastic_work_increment_j;
  result[i++] = value.cumulative_plastic_work_J;
  EXPECT_EQ(i, result.size());
  return result;
}
void SameSection(const fe::ShellBatchLayeredSection& a, const fe::ShellBatchLayeredSection& b) {
  ASSERT_EQ(a.law(), b.law());
  if (a.one_point()) {
    ASSERT_NE(b.one_point(), nullptr);
    EXPECT_EQ(PointValues(*a.one_point()), PointValues(*b.one_point()));
  } else if (a.elastic()) {
    ASSERT_NE(b.elastic(), nullptr);
    EXPECT_EQ(Bytes(*a.elastic()), Bytes(*b.elastic()));
  } else {
    ASSERT_NE(a.plastic(), nullptr);
    ASSERT_NE(b.plastic(), nullptr);
    EXPECT_EQ(Bytes(*a.plastic()), Bytes(*b.plastic()));
  }
}
}
void Same(const Frame& a, const Frame& b) {
  for (unsigned e = 0; e < Parents; ++e) {
    EXPECT_EQ(Bytes(a.qforce[e]), Bytes(b.qforce[e]));
    mixed::to::Exact(a.tforce[e], b.tforce[e]);
    SameSection(a.qsection[e], b.qsection[e]);
    SameSection(a.tsection[e], b.tsection[e]);
  }
}
void NativeAgreement(const Frame& before, const Frame& next, const pure::Fixture& fixture,
    const pure::NativeState& native) {
  const auto* point = next.tsection[1].one_point();
  const auto* old = before.tsection[1].one_point();
  ASSERT_NE(point, nullptr);
  ASSERT_NE(old, nullptr);
  const auto& force = next.tforce[1];
  t3::OnePointForceTrial packet;
  const t3::OnePointHistoryValues values{force.proposed_history.data(), point->point.saved,
      point->point.failure.history, point->cumulative_plastic_work_J};
  ASSERT_EQ(t3::PrepareOnePointLaw44History(fixture.reference, fixture.material, fixture.failure,
      values, force.proposed_history.stamp(), packet.proposed_history), t3::Status::kSuccess);
  packet.kinematics = force.kinematics;
  packet.diagnostics = force.diagnostics;
  packet.point = point->point;
  packet.plastic_work_increment_j = point->cumulative_plastic_work_J - old->cumulative_plastic_work_J;
  packet.removed_now = old->point.failure.history.point_active && !point->point.failure.history.point_active;
  for (unsigned n = 0; n < 3; ++n) {
    packet.internal_force[n] = force.internal_force[n];
    packet.internal_couple[n] = force.internal_couple[n];
  }
  const double factor = force.kinematics.dt / force.kinematics.area;
  for (unsigned c = 0; c < 8; ++c) packet.strain_curvature_increment[c] = force.kinematics.raw_rate[c] * factor;
  pure::CompareNative(packet, native);
}
} // namespace t3_one_point_resident_test
