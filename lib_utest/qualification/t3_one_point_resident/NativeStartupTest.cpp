#include "../t3_one_point/Fixture.h"
#include "lib_utest/qualification/native/t3/T3Reference.h"

namespace {
TEST(T3OnePointResidentNative, OriginalEid2357656UsesNativeOrdinaryStartupMassInertiaAndZeros) {
  namespace native = tl::qualification::t3;
  const t3_one_point_test::Fixture fixture;
  const auto& actual = fixture.reference;
  native::ReferenceInput input;
  for (unsigned n = 0; n < 3; ++n) {
    input.position[n] = actual.input.position[n];
    input.node_ids[n] = actual.input.node_ids[n];
  }
  input.density = actual.input.density;
  input.thickness = actual.input.thickness;
  input.young_modulus = actual.input.young_modulus;
  input.poisson_ratio = actual.input.poisson_ratio;
  native::Reference reference;
  ASSERT_EQ(native::Initialize(input, reference), native::Status::kSuccess);
  const auto& expected = reference.data();
  auto same = [](double a, double b) {
    EXPECT_NEAR(a, b, 8e-14 * std::max(std::abs(a), std::abs(b)));
  };
  for (unsigned i = 0; i < 9; ++i) same(actual.frame.v[i], expected.frame.v[i]);
  same(actual.area, expected.area);
  for (unsigned n = 0; n < 3; ++n) {
    same(actual.local_position[n].x, expected.local_position[n].x);
    same(actual.local_position[n].y, expected.local_position[n].y);
    same(actual.local_position[n].z, expected.local_position[n].z);
    same(actual.angle_cosine[n], expected.angle_cosine[n]);
    same(actual.angle_weight[n], expected.angle_weight[n]);
    same(actual.nodal_mass[n], expected.nodal_mass[n]);
    same(actual.isotropic_inertia[n], expected.isotropic_inertia[n]);
    same(actual.physical_inertia[n], expected.physical_inertia[n]);
    same(actual.added_inertia[n], expected.added_inertia[n]);
    EXPECT_EQ(actual.startup_derivative[n], 0.);
    EXPECT_EQ(expected.startup_derivative[n], 0.);
  }
  same(actual.element_mass, expected.element_mass);
  same(actual.element_isotropic_inertia, expected.element_isotropic_inertia);
  same(actual.element_physical_inertia, expected.element_physical_inertia);
  same(actual.element_added_inertia, expected.element_added_inertia);
  same(actual.characteristic_length, expected.characteristic_length);
  // The selected ordinary C3INMAS and zero-derivative branches do not depend
  // on NPT. Starter stiffness/SSP are not outputs of this reference oracle.
}
}
