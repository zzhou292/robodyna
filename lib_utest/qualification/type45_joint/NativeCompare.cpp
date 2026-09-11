// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include <algorithm>

namespace type45_test {
namespace {
void NearValue(double actual,double expected) {
  EXPECT_NEAR(actual,expected,2e-12*std::max(1.,std::abs(expected)));
}
void Vector(Vec3 actual,const double* expected,double scale) {
  NearValue(actual.x,expected[0]*scale);
  NearValue(actual.y,expected[1]*scale);
  NearValue(actual.z,expected[2]*scale);
}
} // namespace
void CompareReference(const Fixture& fixture,const Reference& reference,const NativeOracle& native) {
  const auto& initial=reference.startup();
  const auto& a=reference.automatic_stiffness();
  const auto& observed=native.startup.values;
  NearValue(initial.maximum_translation_n_m,observed[0]*native.mass);
  NearValue(initial.maximum_rotation_nm,observed[1]*native.inertia);
  NearValue(initial.maximum_viscosity_n_s_m,observed[2]*native.mass);
  NearValue(initial.maximum_rotational_viscosity_nm_s,observed[3]*native.inertia);
  NearValue(a.blocked_translation_n_m,observed[4]*native.mass);
  NearValue(a.blocked_rotation_nm,observed[5]*native.inertia);
  NearValue(a.unconstrained_translation_limit_n_m,observed[6]*native.mass);
  NearValue(a.unconstrained_rotation_limit_nm,observed[7]*native.inertia);
  EXPECT_EQ(a.raised_to_structural_stiffness,observed[8]!=0);
  for (unsigned i=0; i<2; ++i) {
    NearValue(reference.damping(i).mass_kg,observed[9+i]*native.mass);
    NearValue(reference.damping(i).mean_principal_inertia_kg_m2,observed[11+i]*native.inertia);
  }
  double expected[39]{};
  const Vec3 separation=reference.local_separation_m();
  for (unsigned i=0; i<3; ++i) {
    expected[i]=tl::fea::type45::detail::Get(separation,i)/native.length;
    expected[18+i]=tl::fea::type45::detail::Get(reference.stiffness().translation,i)/native.mass;
    expected[30+i]=tl::fea::type45::detail::Get(reference.stiffness().rotation,i)/native.inertia;
    for (unsigned j=0; j<3; ++j) expected[21+3*i+j]=reference.frame().v[3*j+i];
  }
  expected[16]=a.blocked_translation_n_m/native.mass;
  expected[17]=a.blocked_rotation_nm/native.inertia;
  for (unsigned i=0; i<2; ++i) {
    expected[33+i]=reference.damping(i).mass_kg/native.mass;
    expected[35+i]=reference.damping(i).mean_principal_inertia_kg_m2/native.inertia;
    expected[37+i]=fixture.context.main[i].role==EndpointRole::RigidMember ? i+1 : 0;
  }
  for (unsigned i=0; i<39; ++i) {
    SCOPED_TRACE(i);
    NearValue(native.state.uvar[i],expected[i]);
  }
}
void CompareStep(const Evaluation& actual,const NativeOracle& native,const type45_native::Step& expected) {
  const auto& h=actual.history.values();
  for (unsigned row=0; row<3; ++row)
    for (unsigned col=0; col<3; ++col)
      NearValue(h.frame.v[3*row+col],native.state.uvar[21+3*col+row]);
  Vector(h.local_displacement_m,native.state.history.data(),native.length);
  Vector(h.relative_rotation_rad,native.state.history.data()+3,1);
  Vector(h.local_force_n,native.state.history.data()+6,native.force);
  Vector(h.local_couple_nm,native.state.history.data()+9,native.inertia);
  NearValue(h.internal_work_j,native.state.history[12]*native.inertia);
  for (unsigned i=0; i<2; ++i) {
    Vector(actual.endpoint[i].force_n,expected.values.data()+8*i,native.force);
    Vector(actual.endpoint[i].couple_nm,expected.values.data()+8*i+3,native.inertia);
    NearValue(actual.endpoint[i].translational_stiffness_n_m,expected.values[8*i+6]*native.mass);
    NearValue(actual.endpoint[i].rotational_stiffness_nm,expected.values[8*i+7]*native.inertia);
  }
  NearValue(actual.diagnostics.maximum_stiffness_n_m,expected.values[16]*native.mass);
  NearValue(actual.diagnostics.maximum_rotational_stiffness_nm,expected.values[17]*native.inertia);
  NearValue(actual.diagnostics.damping_n_s_m,expected.values[18]*native.mass);
  NearValue(actual.diagnostics.rotational_damping_nm_s,expected.values[19]*native.inertia);
  Vector(actual.diagnostics.local_separation_m,expected.values.data()+20,native.length);
  EXPECT_EQ(expected.values[23],0); // Native physical joint mass/inertia outputs.
  EXPECT_EQ(expected.values[24],0);
}
} // namespace type45_test
