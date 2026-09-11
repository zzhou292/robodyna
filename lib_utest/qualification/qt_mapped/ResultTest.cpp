// SPDX-License-Identifier: MIT
#include "../qbat_binding/Fixture.h"
#include "lib_src/elements/qeph/mapped/Result.h"
#include "lib_src/elements/t3/mapped/Result.h"
#include "lib_src/elements/qeph/QephStartup.h"

namespace qt_mapped_test {
namespace fe=tl::fea;
TEST(QtMappedResult, SkinAdvancesOnlyEndpointAndRejectsAnyMechanicalField) {
  qbat_binding_test::Fixture source;
  fe::qeph::ReferenceData reference;
  ASSERT_EQ(fe::qeph::InitializeReference(source.q[0].reference,reference),fe::qeph::Status::kSuccess);
  fe::qeph::ForceTrial base,next;
  ASSERT_EQ(fe::qeph::InitializeHistory(reference,{0,0},base.proposed_history),fe::qeph::Status::kSuccess);
  fe::qeph::PrescribedInterval interval;
  interval.dt=1e-7;
  interval.sample_index=1;
  ASSERT_EQ(fe::qeph::mapped::AdvanceSkin(reference,base,interval,next),fe::qeph::Status::kSuccess);
  ASSERT_TRUE(fe::qeph::mapped::ValidResult(reference,next,interval.dt,1,true));
  EXPECT_EQ(next.kinematics.area,0);
  EXPECT_EQ(next.diagnostics.unscaled_element_dt,0);
  const auto original=qbat_binding_test::Bytes(next);
  base.internal_force[3].z=1e-300;
  EXPECT_NE(fe::qeph::mapped::AdvanceSkin(reference,base,interval,next),fe::qeph::Status::kSuccess);
  EXPECT_EQ(qbat_binding_test::Bytes(next),original);
  base.internal_force[3].z=0;
  ASSERT_EQ(fe::qeph::mapped::AdvanceSkin(reference,base,interval,next),fe::qeph::Status::kSuccess);
  EXPECT_TRUE(fe::qeph::mapped::ValidResult(reference,next,interval.dt,1,true));
}
TEST(QtMappedResult, TriangleSkinRetainsSourceIdentityAndNoConstitutiveAvailability) {
  qbat_binding_test::Fixture source;
  fe::t3::ReferenceData reference;
  ASSERT_EQ(fe::t3::InitializeReference(source.t.reference,reference),fe::t3::Status::kSuccess);
  fe::t3::ForceTrial base,next;
  ASSERT_EQ(fe::t3::InitializeHistory(reference,{0,0},base.proposed_history),fe::t3::Status::kSuccess);
  fe::t3::PrescribedInterval interval;
  interval.dt=1e-7;
  interval.sample_index=1;
  ASSERT_EQ(fe::t3::mapped::AdvanceSkin(reference,base,interval,next),fe::t3::Status::kSuccess);
  ASSERT_TRUE(fe::t3::mapped::ValidResult(reference,next,interval.dt,1,true));
  auto wrong=reference;
  ++wrong.input.node_ids[2];
  EXPECT_FALSE(fe::t3::mapped::ValidResult(wrong,next,interval.dt,1,true));
  next.diagnostics.rotational_stiffness=1;
  EXPECT_FALSE(fe::t3::mapped::ValidResult(reference,next,interval.dt,1,true));
}
} // namespace qt_mapped_test
