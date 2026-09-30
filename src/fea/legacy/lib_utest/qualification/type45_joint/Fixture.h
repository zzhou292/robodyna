// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/type45/Type45Force.h"
#include <gtest/gtest.h>

namespace type45_test {
using namespace tl::fea::type45;
namespace f=tl::math::fixed3;
struct Fixture {
  Property property;
  GeometryInput geometry{71,{101,102,103},{{0,0,0},{.03,.02,-.01},{1,0,0}}};
  DampingEndpoint damping[2]{{2,.03},{3,.07}};
  AutomaticStiffnessContext context{1e-4,{
    {EndpointRole::RigidMember,201,{-.02,.01,0},2,.01,120,4},
    {EndpointRole::RigidMember,202,{.05,.04,.02},3,.02,150,5}}};
  explicit Fixture(Kind kind=Kind::Revolute) {
    property.kind=kind;
    if (kind==Kind::Spherical) {
      geometry.source_node_id[2]=0;
      geometry.position_m[2]={};
    }
  }
  Reference Prepare() const {
    Reference result;
    EXPECT_EQ(Reference::Prepare(property,geometry,damping,context,result),Status::Success);
    return result;
  }
  Interval Step(const History& history, double dt=1e-4) const {
    return {{geometry.position_m[0],geometry.position_m[1]},{{},{}},
            history.stamp().time_s,dt,history.stamp().sample_index+1};
  }
};
inline void Near(Vec3 actual, Vec3 expected, double tolerance) {
  EXPECT_NEAR(actual.x,expected.x,tolerance);
  EXPECT_NEAR(actual.y,expected.y,tolerance);
  EXPECT_NEAR(actual.z,expected.z,tolerance);
}
inline void Same(const Evaluation& a, const Evaluation& b) {
  EXPECT_EQ(a.history.ready(),b.history.ready());
  const auto& x=a.history.values();
  const auto& y=b.history.values();
  for (unsigned i=0; i<9; ++i) EXPECT_DOUBLE_EQ(x.frame.v[i],y.frame.v[i]);
  Near(x.local_displacement_m,y.local_displacement_m,0);
  Near(x.relative_rotation_rad,y.relative_rotation_rad,0);
  Near(x.local_force_n,y.local_force_n,0);
  Near(x.local_couple_nm,y.local_couple_nm,0);
  EXPECT_DOUBLE_EQ(x.internal_work_j,y.internal_work_j);
  EXPECT_EQ(a.history.stamp().sample_index,b.history.stamp().sample_index);
  EXPECT_DOUBLE_EQ(a.history.stamp().time_s,b.history.stamp().time_s);
  for (unsigned i=0; i<2; ++i) {
    Near(a.endpoint[i].force_n,b.endpoint[i].force_n,0);
    Near(a.endpoint[i].couple_nm,b.endpoint[i].couple_nm,0);
    EXPECT_DOUBLE_EQ(a.endpoint[i].translational_stiffness_n_m,b.endpoint[i].translational_stiffness_n_m);
    EXPECT_DOUBLE_EQ(a.endpoint[i].rotational_stiffness_nm,b.endpoint[i].rotational_stiffness_nm);
  }
  Near(a.diagnostics.local_separation_m,b.diagnostics.local_separation_m,0);
  Near(a.diagnostics.local_velocity_m_s,b.diagnostics.local_velocity_m_s,0);
  Near(a.diagnostics.relative_rate_rad_s,b.diagnostics.relative_rate_rad_s,0);
  EXPECT_DOUBLE_EQ(a.diagnostics.harmonic_mass_kg,b.diagnostics.harmonic_mass_kg);
  EXPECT_DOUBLE_EQ(a.diagnostics.harmonic_inertia_kg_m2,b.diagnostics.harmonic_inertia_kg_m2);
  EXPECT_DOUBLE_EQ(a.diagnostics.maximum_stiffness_n_m,b.diagnostics.maximum_stiffness_n_m);
  EXPECT_DOUBLE_EQ(a.diagnostics.maximum_rotational_stiffness_nm,b.diagnostics.maximum_rotational_stiffness_nm);
  EXPECT_DOUBLE_EQ(a.diagnostics.damping_n_s_m,b.diagnostics.damping_n_s_m);
  EXPECT_DOUBLE_EQ(a.diagnostics.rotational_damping_nm_s,b.diagnostics.rotational_damping_nm_s);
  EXPECT_DOUBLE_EQ(a.diagnostics.internal_work_increment_j,b.diagnostics.internal_work_increment_j);
}
} // namespace type45_test
