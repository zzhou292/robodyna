#pragma once
#include "Truth.h"
#include "lib_src/collision/NodalWallContactDiagnostics.h"
namespace wall_observer_test {
inline void CheckIntervalTruth(const d::Storage& s,const tl::fea::NodalPreparedView& view) {
  High kick=0,drift=0,impulse[2]{},my[2]{},mz[2]{},physical_drift[2]{},potential_delta[2]{};
  auto add_scaled=[](High (&sum)[2],double lower,double upper,High scale) {
    sum[0]+=High(scale>=0?lower:upper)*scale;sum[1]+=High(scale>=0?upper:lower)*scale;
  };
  for(unsigned i=0;i<s.model.node_count;++i) {
    const auto& n=s.base.nodes[i];const auto index=3*n.node;
    const High force=n.force_world.x;
    kick+=High(view.kick_dt)*force*(High(view.base_kinematics.velocity_xyz[index])+High(view.kinematics.velocity_xyz[index]))/2;
    drift+=force*(High(view.kinematics.position_xyz[index])-High(view.base_kinematics.position_xyz[index]));
    add_scaled(impulse,n.force.lower,n.force.upper,High(view.kick_dt));
    add_scaled(my,n.force.lower,n.force.upper,High(view.kick_dt)*High(n.wall_point.z));
    add_scaled(mz,n.force.lower,n.force.upper,-High(view.kick_dt)*High(n.wall_point.y));
    add_scaled(physical_drift,n.force.lower,n.force.upper,
        -(High(view.kinematics.position_xyz[index])-High(view.base_kinematics.position_xyz[index])));
    potential_delta[0]+=High(s.result.nodes[i].potential.lower)-High(n.potential.upper);
    potential_delta[1]+=High(s.result.nodes[i].potential.upper)-High(n.potential.lower);
  }
  const auto& actual=s.result.diagnostics;
  auto bounded=[](double value,High truth,double radius) {
    EXPECT_TRUE(Abs(High(value)-truth)<=High(radius));
    const double corrupt=::nextafter(double(truth+4*High(radius)+1),INFINITY);
    EXPECT_TRUE(Abs(High(corrupt)-truth)>High(radius));
  };
  bounded(actual.kick_work,kick,actual.kick_work_roundoff);
  bounded(actual.drift_work,drift,actual.drift_work_roundoff);
  for(unsigned endpoint=0;endpoint<2;++endpoint) {
    bounded(actual.conservative_defect,potential_delta[endpoint]+physical_drift[endpoint],actual.work_uncertainty);
    bounded(actual.wall_kick_impulse,impulse[endpoint],actual.wall_kick_impulse_error);
    bounded(actual.wall_kick_moment.y,my[endpoint],actual.wall_kick_moment_error.y);
    bounded(actual.wall_kick_moment.z,mz[endpoint],actual.wall_kick_moment_error.z);
  }
}
} // namespace wall_observer_test
