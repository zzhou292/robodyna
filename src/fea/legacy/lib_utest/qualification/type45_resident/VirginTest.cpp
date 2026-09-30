// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../type45_joint/Fixture.h"
#include "lib_src/elements/type45/resident/Virgin.h"

namespace type45_test {
namespace virgin=tl::fea::type45::resident_detail;
TEST(Type45Virgin,ThreeKindsKeepZeroPreAutomaticCacheSeparateFromAutomaticReference) {
  for(auto kind:{Kind::Spherical,Kind::Revolute,Kind::Cylindrical}) {
    Fixture fixture(kind); fixture.property.automatic_stiffness_scale=.01;
    virgin::VirginCache cache;
    ASSERT_EQ(virgin::PrepareVirgin(fixture.property,fixture.geometry,cache),Status::Success);
    Near(cache.history.local_displacement_m,{},0); Near(cache.history.relative_rotation_rad,{},0);
    Near(cache.history.local_force_n,{},0); Near(cache.history.local_couple_nm,{},0);
    EXPECT_EQ(cache.history.internal_work_j,0);
    const auto reference=fixture.Prepare();
    EXPECT_GT(reference.automatic_stiffness().blocked_translation_n_m,0);
    for(unsigned end=0;end<2;++end) {
      Near(cache.endpoint[end].force_n,{},0); Near(cache.endpoint[end].couple_nm,{},0);
      EXPECT_EQ(cache.endpoint[end].translational_stiffness_n_m,0);
      EXPECT_EQ(cache.endpoint[end].rotational_stiffness_nm,0);
    }
    History history; ASSERT_EQ(History::Initialize(reference,history),Status::Success);
    Evaluation post_automatic;
    ASSERT_EQ(Evaluate(reference,history,fixture.Step(history),post_automatic),Status::Success);
    EXPECT_GT(post_automatic.endpoint[0].translational_stiffness_n_m,0);
    // Ordinary Evaluate keeps its positive-increment guard.
    auto zero=fixture.Step(history); zero.dt_s=0;
    EXPECT_EQ(Evaluate(reference,history,zero,post_automatic),Status::StaleInterval);
  }
}
TEST(Type45Virgin,LateGeometryOrNonzeroFreeCoefficientKeepsOldOutputThenRetries) {
  Fixture fixture; virgin::VirginCache cache;
  ASSERT_EQ(virgin::PrepareVirgin(fixture.property,fixture.geometry,cache),Status::Success);
  cache.endpoint[1].force_n={7,8,9};
  auto bad=fixture.geometry; bad.position_m[2]={};
  EXPECT_EQ(virgin::PrepareVirgin(fixture.property,bad,cache),Status::InvalidGeometry);
  Near(cache.endpoint[1].force_n,{7,8,9},0);
  auto property=fixture.property; property.free_stiffness.rotation.x=1;
  EXPECT_EQ(virgin::PrepareVirgin(property,fixture.geometry,cache),Status::InvalidProperty);
  Near(cache.endpoint[1].force_n,{7,8,9},0);
  ASSERT_EQ(virgin::PrepareVirgin(fixture.property,fixture.geometry,cache),Status::Success);
  Near(cache.endpoint[1].force_n,{},0);
}
} // namespace type45_test
