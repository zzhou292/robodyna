#pragma once
#include "../mapped_wall_observers/Fixture.h"
#include "../mapped_wall_observers/ExactChecks.h"
#include "lib_src/collision/nodal_wall_mapped/IntervalFinalizer.h"
#include "SerialInterval.h"
namespace wall_interval_test {
namespace c = tlfea::contact;
namespace m = c::nodal_wall_mapped;
namespace device = c::nodal_wall_device_detail;
namespace check = wall_observer_test;
using Code = c::NodalWallDeviceStatus;
struct Fixture {
  check::Fixture base, current;
  std::vector<double> base_velocity, velocity, addition;
  tl::fea::NodalPreparedView view;
  c::NodalWallDiagnostics seed;
  explicit Fixture(unsigned count = 263) : base(count), current(count),
      base_velocity(3*count), velocity(3*count), addition(count, 1e-12) {
    for (unsigned i = 0; i < count; ++i) {
      const auto n = base.nodes[i].node;
      base_velocity[3*n] = .002*double(int(i%7)-3);
      velocity[3*n] = base_velocity[3*n]+.0003;
      current.positions[3*n] = base.positions[3*n]+.00001;
      for (auto* source : {&base, &current}) {
        auto& node = source->nodes[i];
        node.wall_point = {0, .5+double(i%3), .25+double(i%5)};
        node.wall_moment = {0, node.wall_point.z*node.force.value, -node.wall_point.y*node.force.value};
      }
    }
    view.kick_dt = .001;
    view.base_kinematics = base.Kinematics();
    view.base_kinematics.velocity_xyz = base_velocity.data();
    view.kinematics = current.Kinematics();
    view.kinematics.velocity_xyz = velocity.data();
    RecomputeDiagnostics();
  }
  device::Storage& Storage() { return current.storage; }
  void RecomputeDiagnostics() {
    auto& s = Storage();
    s.base.nodes = base.nodes.data();
    s.addition_error = addition.data();
    s.base.diagnostics = base.Serial();
    seed = current.Serial();
    Reset();
  }
  void Reset() { Storage().control = {}; Storage().result.diagnostics = seed; }
  m::IntervalSummary Reduce() {
    const auto blocks = m::ObserverBlocks(base.nodes.size());
    std::vector<m::IntervalSummary> partial(blocks);
    std::array<m::IntervalSummary, m::ObserverThreads> lanes{};
    auto fold = [&] {
      for (unsigned offset = m::ObserverThreads/2; offset; offset /= 2)
        for (unsigned t = 0; t < offset; ++t) m::MergeIntervals(lanes[t], lanes[t+offset]);
    };
    for (unsigned b = 0; b < blocks; ++b) {
      lanes = {};
      for (unsigned t = 0; t < m::ObserverThreads; ++t)
        for (unsigned n = b*m::ObserverThreads+t; n < base.nodes.size(); n += blocks*m::ObserverThreads)
          m::ObserveIntervalNode(Storage(), view, n, lanes[t]);
      fold();
      partial[b] = lanes[0];
    }
    lanes = {};
    for (unsigned t = 0; t < m::ObserverThreads; ++t)
      for (unsigned b = t; b < blocks; b += m::ObserverThreads) m::MergeIntervals(lanes[t], partial[b]);
    fold();
    return lanes[0];
  }
  bool Serial() { Reset(); return c::wall_interval_frozen::MeasureInterval(Storage(), view); }
  bool Staged() { Reset(); return m::FinalizeInterval(Storage(), view, Reduce()); }
};
inline void ExactDiagnostics(c::NodalWallDiagnostics a, c::NodalWallDiagnostics b) {
  double c::NodalWallDiagnostics::* const fields[]{
      &c::NodalWallDiagnostics::base_potential, &c::NodalWallDiagnostics::base_potential_error,
      &c::NodalWallDiagnostics::potential_increment, &c::NodalWallDiagnostics::kick_work,
      &c::NodalWallDiagnostics::kick_work_roundoff, &c::NodalWallDiagnostics::drift_work,
      &c::NodalWallDiagnostics::drift_work_roundoff, &c::NodalWallDiagnostics::conservative_defect,
      &c::NodalWallDiagnostics::work_uncertainty, &c::NodalWallDiagnostics::quadratic_work_upper,
      &c::NodalWallDiagnostics::wall_kick_impulse, &c::NodalWallDiagnostics::wall_kick_impulse_error};
  for (auto field : fields) { check::Exact(a.*field, b.*field); a.*field = b.*field = 0; }
  check::Exact(a.wall_kick_moment, b.wall_kick_moment);
  check::Exact(a.wall_kick_moment_error, b.wall_kick_moment_error);
  a.wall_kick_moment = b.wall_kick_moment = {};
  a.wall_kick_moment_error = b.wall_kick_moment_error = {};
  check::ExactDiagnostics(a, b);
}
inline void FallbackMatches(Fixture& f) {
  auto next = f.seed;
  ASSERT_FALSE(m::ApplyInterval(f.Reduce(), f.Storage(), f.view, next));
  const bool serial_ok = f.Serial();
  const auto serial = f.Storage().result.diagnostics;
  const auto control = f.Storage().control;
  EXPECT_EQ(f.Staged(), serial_ok);
  ExactDiagnostics(f.Storage().result.diagnostics, serial);
  EXPECT_EQ(f.Storage().control.status, control.status);
  EXPECT_EQ(f.Storage().control.node, control.node);
  EXPECT_EQ(f.Storage().control.parent, control.parent);
}
void CheckEndpointEnclosure(Fixture&);
void CheckRoundedWork(const Fixture&, const c::NodalWallDiagnostics&);
void CheckPhysicalTruth(const device::Storage&, const tl::fea::NodalPreparedView&);

} // namespace wall_interval_test
