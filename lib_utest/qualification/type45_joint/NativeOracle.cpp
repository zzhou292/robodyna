// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include <climits>

namespace type45_test {
NativeOracle::NativeOracle(const Fixture& fixture,int& status) {
  kind=static_cast<int>(fixture.property.kind);
  if (fixture.property.working_units==WorkingUnits::MillimetreTonneSecond) {
    length=.001;
    mass=1000;
  }
  inertia=mass*length*length;
  force=mass*length;
  property[0]=fixture.property.automatic_stiffness_scale;
  property[1]=fixture.property.critical_damping_ratio;
  for (unsigned i=0; i<6; ++i) {
    const double scale=i<3 ? mass : inertia;
    property[2+i]=tl::fea::type45::detail::Get(fixture.property.free_stiffness,i)/scale;
    property[8+i]=tl::fea::type45::detail::Get(fixture.property.free_viscosity,i)/scale;
  }
  double positions[15]{},damping[4]{},coefficient[8]{};
  int roles[2]{};
  for (unsigned i=0; i<5; ++i) {
    const Vec3 position=i<3 ? fixture.geometry.position_m[i] : fixture.context.main[i-3].position_m;
    positions[3*i]=position.x/length;
    positions[3*i+1]=position.y/length;
    positions[3*i+2]=position.z/length;
  }
  for (unsigned i=0; i<2; ++i) {
    roles[i]=static_cast<int>(fixture.context.main[i].role);
    damping[2*i]=fixture.damping[i].mass_kg/mass;
    damping[2*i+1]=fixture.damping[i].mean_principal_inertia_kg_m2/inertia;
    const auto& node=fixture.context.main[i];
    coefficient[4*i]=node.mass_kg/mass;
    coefficient[4*i+1]=node.inertia_kg_m2/inertia;
    coefficient[4*i+2]=node.translational_stiffness_n_m/mass;
    coefficient[4*i+3]=node.rotational_stiffness_nm/inertia;
  }
  type45_native::type45_native_startup(&kind,property.data(),positions,roles,damping,
    coefficient,&fixture.context.target_dt_s,state.uvar.data(),startup.values.data(),&status);
}
bool NativeOracle::Step(const Interval& interval,type45_native::Step& output) {
  if (interval.sample_index>INT_MAX) return false;
  const int cycle=static_cast<int>(interval.sample_index);
  const double time=interval.base_time_s+interval.dt_s;
  double positions[6]{},spin[6]{};
  for (unsigned i=0; i<2; ++i) {
    for (unsigned a=0; a<3; ++a) {
      positions[3*i+a]=tl::fea::type45::detail::Get(interval.position_m[i],a)/length;
      spin[3*i+a]=tl::fea::type45::detail::Get(interval.angular_velocity_rad_s[i],a);
    }
  }
  int status=-1;
  type45_native::type45_native_step(&kind,property.data(),positions,spin,&time,&interval.dt_s,
    &cycle,state.uvar.data(),state.history.data(),output.values.data(),&status);
  return status==0;
}
} // namespace type45_test
