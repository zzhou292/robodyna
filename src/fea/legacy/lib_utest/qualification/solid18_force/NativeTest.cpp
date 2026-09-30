// SPDX-License-Identifier: AGPL-3.0-or-later
#include "PacketValues.h"

namespace solid18_force_test {
namespace {
s::PrescribedInterval RotatingPath(const s::Reference& reference, unsigned step) {
  auto motion = Path(reference,step);
  constexpr double spin = 3000;
  const double end = motion.base_time_s+motion.dt_s;
  const double mid = motion.base_time_s+.5*motion.dt_s;
  const double end_cos = std::cos(spin*end);
  const double end_sin = std::sin(spin*end);
  const double mid_cos = std::cos(spin*mid);
  const double mid_sin = std::sin(spin*mid);
  const double amplitude = .05*std::sin(2*3.141592653589793*mid/(320*motion.dt_s));
  for (unsigned n = 0; n < 8; ++n) {
    const auto x = reference.input().position_m[n];
    const s::Vec3 shape{x.x+.4*x.y+.2*x.x*x.z/.006,-.3*x.y+.15*x.z,-.2*x.z+.13*x.x};
    const s::Vec3 midpoint{x.x+amplitude*shape.x,x.y+amplitude*shape.y,x.z+amplitude*shape.z};
    const auto p = motion.position_endpoint_m[n];
    const auto v = motion.velocity_midpoint_m_s[n];
    const double mx = mid_cos*midpoint.x-mid_sin*midpoint.y;
    const double my = mid_sin*midpoint.x+mid_cos*midpoint.y;
    motion.position_endpoint_m[n] = {end_cos*p.x-end_sin*p.y+.1*end,
                                    end_sin*p.x+end_cos*p.y-.2*end,p.z+.3*end};
    motion.velocity_midpoint_m_s[n] = {mid_cos*v.x-mid_sin*v.y-spin*my+.1,
                                      mid_sin*v.x+mid_cos*v.y+spin*mx-.2,v.z+.3};
  }
  return motion;
}
void RunNativeTrajectory(bool rotating) {
  const auto reference = Reference();
  const auto material = Material();
  auto native = NativeInitial(reference.input());
  s::History accepted;
  ASSERT_EQ(s::InitializeHistory(reference,material,accepted),s::Status::Success);
  for (unsigned n = 0; n < 8; ++n) ASSERT_EQ(reference.source_slot(n),native.source_slot[n]);
  bool yielded = false;
  bool unloaded = false;
  double prior_pla = 0;
  double peak_q = 0;
  for (unsigned step = 0; step < 400; ++step) {
    SCOPED_TRACE(step);
    const auto interval = rotating ? RotatingPath(reference,step) : Path(reference,step);
    const auto expected = Native(material,native,interval);
    ASSERT_EQ(expected.status,0);
    s::ForceTrial actual;
    ASSERT_EQ(s::EvaluateForce(reference,accepted,interval,material,actual),s::Status::Success);
    ASSERT_TRUE(Agree(actual,expected));
    ASSERT_EQ(actual.proposed_history.stamp().sample_index,step+1);
    ASSERT_DOUBLE_EQ(actual.proposed_history.stamp().time_s,interval.base_time_s+interval.dt_s);
    const double plastic = expected.next.point[12];
    yielded |= plastic > .001;
    unloaded |= step > 50 && plastic == prior_pla && expected.observation[26] < 0;
    prior_pla = plastic;
    for (unsigned ip = 0; ip < 8; ++ip) {
      peak_q = std::max(peak_q,expected.next.point[20*ip+19]);
      EXPECT_EQ(expected.next.point[20*ip+18],native.point[20*ip+18]);
    }
    accepted = actual.proposed_history;
    native = expected.next; // Independent native accepted history, never reset from TL.
  }
  EXPECT_TRUE(yielded);
  EXPECT_TRUE(unloaded);
  EXPECT_GT(peak_q,0);
  EXPECT_GT(native.global[9],.01);
}
}

TEST(Solid18ForceNative, EightIndependentHistoriesCyclicUnloadReloadAndNativeWork) { RunNativeTrajectory(false); }
TEST(Solid18ForceNative, RotatingYieldedCallerRetainsActualEndpointAndMidpointPhases) { RunNativeTrajectory(true); }

TEST(Solid18ForceNative, SixGradientChannelsAndHydrostaticCompressionUseNativeVolumePressure) {
  const auto reference = Reference(false);
  const auto material = Material();
  constexpr double dt = 1.0/1048576;
  for (unsigned channel = 0; channel < 7; ++channel) {
    SCOPED_TRACE(channel);
    auto native = NativeInitial(reference.input());
    s::History accepted;
    ASSERT_EQ(s::InitializeHistory(reference,material,accepted),s::Status::Success);
    for (unsigned step = 0; step < 12; ++step) {
      s::PrescribedInterval interval;
      interval.base_time_s = step*dt;
      interval.dt_s = dt;
      interval.sample_index = step+1;
      const double time = (step+1)*dt;
      for (unsigned n = 0; n < 8; ++n) {
        const auto x = reference.input().position_m[n];
        s::Vec3 v{};
        if (channel == 0) v.x = 800*x.x;
        if (channel == 1) v.y = 800*x.y;
        if (channel == 2) v.z = 800*x.z;
        if (channel == 3) v.x = 800*x.y;
        if (channel == 4) v.y = 800*x.z;
        if (channel == 5) v.z = 800*x.x;
        if (channel == 6) v = {-100*x.x,-100*x.y,-100*x.z};
        interval.position_endpoint_m[n] = {x.x+time*v.x,x.y+time*v.y,x.z+time*v.z};
        interval.velocity_midpoint_m_s[n] = v;
      }
      const auto expected = Native(material,native,interval);
      ASSERT_EQ(expected.status,0);
      s::ForceTrial actual;
      ASSERT_EQ(s::EvaluateForce(reference,accepted,interval,material,actual),s::Status::Success);
      ASSERT_TRUE(Agree(actual,expected));
      if (channel == 6) {
        EXPECT_LT(expected.next.point[0],0);
        EXPECT_GT(expected.next.point[19],0);
      }
      accepted = actual.proposed_history;
      native = expected.next;
    }
  }
}

TEST(Solid18ForceNative, PointSelectionTieAndZeroUseActualNativeVisitation) {
  const auto reference = Reference();
  const auto material = Material();
  auto native = NativeInitial(reference.input());
  s::History initial;
  ASSERT_EQ(s::InitializeHistory(reference,material,initial),s::Status::Success);
  auto prescribed = initial.data();
  prescribed.global.plastic_strain = .02;
  native.global[6] = .02;
  for (unsigned ip = 0; ip < 8; ++ip) {
    const double plastic = ip == 7 ? 0 : .01;
    prescribed.point[ip].material.point.plastic_strain = plastic;
    prescribed.point[ip].material.point.stress_pa[0] = 1e7;
    native.point[20*ip+12] = plastic;
    native.point[20*ip] = 1e7;
  }
  s::History accepted;
  ASSERT_EQ(s::PreparePrescribedHistory(reference,material,prescribed,{},accepted),s::Status::Success);
  const auto expected = Native(material,native,Path(reference,0));
  ASSERT_EQ(expected.status,0);
  s::ForceTrial actual;
  ASSERT_EQ(s::EvaluateForce(reference,accepted,Path(reference,0),material,actual),s::Status::Success);
  ASSERT_TRUE(Agree(actual,expected));
  EXPECT_EQ(expected.diagnostics[2],3);
  EXPECT_GT(expected.diagnostics[0],0);
}

TEST(Solid18ForceNative, ResetHistoryAndWrongVelocityPhaseAreDetectableControls) {
  const auto reference = Reference();
  const auto material = Material();
  auto native = NativeInitial(reference.input());
  for (unsigned step = 0; step < 120; ++step) {
    const auto trial = Native(material,native,Path(reference,step));
    ASSERT_EQ(trial.status,0);
    native = trial.next;
  }
  const auto interval = Path(reference,120);
  const auto expected = Native(material,native,interval);
  ASSERT_EQ(expected.status,0);
  auto reset = native;
  for (unsigned ip = 0; ip < 8; ++ip) {
    std::fill_n(reset.point.begin()+20*ip,14,0);
  }
  const auto without_history = Native(material,reset,interval);
  ASSERT_EQ(without_history.status,0);
  auto wrong = interval;
  const auto later = Path(reference,121);
  std::copy_n(later.velocity_midpoint_m_s,8,wrong.velocity_midpoint_m_s);
  const auto wrong_phase = Native(material,native,wrong);
  ASSERT_EQ(wrong_phase.status,0);
  double reset_delta = 0;
  double phase_delta = 0;
  for (unsigned n = 0; n < 24; ++n) {
    reset_delta = std::max(reset_delta,std::abs(expected.force[n]-without_history.force[n]));
    phase_delta = std::max(phase_delta,std::abs(expected.force[n]-wrong_phase.force[n]));
  }
  EXPECT_GT(reset_delta,1);
  EXPECT_GT(phase_delta,.001);
}
} // namespace solid18_force_test
