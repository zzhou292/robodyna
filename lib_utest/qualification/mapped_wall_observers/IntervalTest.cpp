#include "IntervalTruth.h"
namespace wall_observer_test {
TEST(MappedWallObserversHost, DerivedIntervalCertificatesEncloseIndependentWorkImpulseAndMoment) {
  Fixture base(263),current(263);
  std::vector<double> base_velocity(3*263),velocity(3*263),error(263);
  for(unsigned i=0;i<263;++i) {
    const auto n=base.nodes[i].node;
    base_velocity[3*n]=double(int(i%7)-3)*.002;velocity[3*n]=base_velocity[3*n]+.0003;
    current.positions[3*n]=base.positions[3*n]+.00001;
    const double u=current.nodes[i].potential.value*1.01;
    ASSERT_TRUE(c::q4_bounds::Certify(u,{::nextafter(u,0),::nextafter(u,INFINITY)},&current.nodes[i].potential));
  }
  tl::fea::NodalPreparedView view;view.kick_dt=.001;
  view.base_kinematics=base.Kinematics();view.base_kinematics.velocity_xyz=base_velocity.data();
  view.kinematics=current.Kinematics();view.kinematics.velocity_xyz=velocity.data();
  auto& s=current.storage;s.base.nodes=base.nodes.data();s.addition_error=error.data();
  s.base.diagnostics=base.Serial();s.result.diagnostics=current.Serial();
  ASSERT_TRUE(d::MeasureInterval(s,view));const auto old=s.result.diagnostics;
  s.base.diagnostics=base.Staged();s.result.diagnostics=current.Staged();
  ASSERT_TRUE(d::MeasureInterval(s,view));CheckIntervalTruth(s,view);
  const auto& now=s.result.diagnostics;
  // These loops and their local arithmetic are outside the observer change.
  EXPECT_EQ(Bits(now.kick_work),Bits(old.kick_work));EXPECT_EQ(Bits(now.kick_work_roundoff),Bits(old.kick_work_roundoff));
  EXPECT_EQ(Bits(now.drift_work),Bits(old.drift_work));EXPECT_EQ(Bits(now.drift_work_roundoff),Bits(old.drift_work_roundoff));
  EXPECT_EQ(Bits(now.quadratic_work_upper),Bits(old.quadratic_work_upper));
}
} // namespace wall_observer_test
