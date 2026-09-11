#include "Fixture.h"
#include "../mapped_wall_observers/IntervalTruth.h"
namespace wall_interval_test {
void CheckEndpointEnclosure(Fixture& f) {
  c::Q4IntegralInterval serial_y, serial_z;
  for (const auto& node : f.base.nodes) {
    c::Q4IntegralInterval term;
    ASSERT_TRUE(device::SignedScale({node.force.lower, node.force.upper}, node.wall_point.z, &term));
    ASSERT_TRUE(c::q4_bounds::Add(serial_y, term, &serial_y));
    ASSERT_TRUE(device::SignedScale({node.force.lower, node.force.upper}, -node.wall_point.y, &term));
    ASSERT_TRUE(c::q4_bounds::Add(serial_z, term, &serial_z));
  }
  const auto tree = f.Reduce();
  double error = 0;
  ASSERT_TRUE(m::SerialEndpointError(f.base.nodes.size(), tree.maximum_term, error));
  const double serial[]{serial_y.lower, serial_y.upper, serial_z.lower, serial_z.upper};
  const double current[]{tree.moment_y.lower, tree.moment_y.upper, tree.moment_z.lower, tree.moment_z.upper};
  for (unsigned i = 0; i < 4; ++i)
    EXPECT_TRUE(check::Abs(check::High(serial[i])-check::High(current[i])) <= check::High(error));
}
void CheckRoundedWork(const Fixture& f, const c::NodalWallDiagnostics& result) {
  check::TruthSum kick, drift;
  kick.Add(f.seed.kick_work);
  drift.Add(f.seed.drift_work);
  check::High quadratic = 0, exact_kick = f.seed.kick_work, exact_drift = f.seed.drift_work;
  for (unsigned i = 0; i < f.base.nodes.size(); ++i) {
    const auto& node = f.base.nodes[i];
    const auto n = 3*node.node;
    const double mean = .5*(f.base_velocity[n]+f.velocity[n]);
    const double dx = f.current.positions[n]-f.base.positions[n];
    kick.Add(f.view.kick_dt*node.force_world.x*mean);
    drift.Add(node.force_world.x*dx);
    const check::High exact_dx = check::High(f.current.positions[n])-check::High(f.base.positions[n]);
    quadratic += check::High(.5)*check::High(node.stiffness.upper)*exact_dx*exact_dx;
    exact_kick += check::High(f.view.kick_dt)*check::High(node.force_world.x)*
        (check::High(f.base_velocity[n])+check::High(f.velocity[n]))/2;
    exact_drift += check::High(node.force_world.x)*exact_dx;
  }
  const auto count = static_cast<unsigned>(f.base.nodes.size()+1);
  EXPECT_TRUE(check::Abs(check::High(result.kick_work)-kick.value) <= kick.Bound(count));
  EXPECT_TRUE(check::Abs(check::High(result.drift_work)-drift.value) <= drift.Bound(count));
  EXPECT_TRUE(check::High(result.quadratic_work_upper) >= quadratic);
  // Seeds are an explicit carried observer offset, not extra physical work.
  // The physical certificate still encloses unseeded work. Adding |seed| gives
  // the corresponding seeded exact-observer enclosure by the triangle bound.
  EXPECT_TRUE(check::Abs(check::High(result.kick_work)-exact_kick) <=
      check::High(result.kick_work_roundoff)+check::Abs(check::High(f.seed.kick_work)));
  EXPECT_TRUE(check::Abs(check::High(result.drift_work)-exact_drift) <=
      check::High(result.drift_work_roundoff)+check::Abs(check::High(f.seed.drift_work)));
}
void CheckPhysicalTruth(const device::Storage& storage, const tl::fea::NodalPreparedView& view) {
  check::CheckIntervalTruth(storage, view);
}
} // namespace wall_interval_test
