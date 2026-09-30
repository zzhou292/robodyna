// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
namespace extended_model_test {
TEST(ExtendedSolidModel, ExplicitProfileAndEveryNewPointerCountPairAreRequired) {
  Fixture f;
  const auto domain = f.Domain();
  for (unsigned choice = 0; choice < 9; ++choice) {
    auto input = f.Input();
    if (choice == 0) input.profile = s::ModelProfile::OriginalThreeFamilies;
    if (choice == 1) input.profile = static_cast<s::ModelProfile>(99);
    if (choice == 2) { input.solid18_law44 = {nullptr,0}; input.solid18_law90 = {nullptr,0}; }
    if (choice == 3) input.solid18_law44 = {nullptr,1};
    if (choice == 4) input.solid18_law90 = {nullptr,1};
    if (choice == 5) input.solid18_law44 = {f.input44.data(),0};
    if (choice == 6) input.solid18_law90 = {f.input90.data(),0};
    if (choice == 7) input.solid18_law44 = {reinterpret_cast<const s::Input18Law44*>(1),1};
    if (choice == 8) input.solid18_law90 = {reinterpret_cast<const s::Input18Law90*>(1),1};
    s::Model model;
    EXPECT_EQ(model.Initialize(domain,input).status,s::ModelStatus::InvalidInput) << choice;
    Empty(model);
    ASSERT_TRUE(model.Initialize(domain,f.Input()));
  }
  for (unsigned family = 0; family < 2; ++family) {
    auto input = f.Input();
    input.solid18 = {nullptr,0}; input.solid24 = {nullptr,0}; input.solid6z = {nullptr,0};
    if (family == 0) input.solid18_law90 = {nullptr,0};
    else input.solid18_law44 = {nullptr,0};
    s::Model model;
    ASSERT_TRUE(model.Initialize(domain,input));
    EXPECT_EQ(model.contributions()->parents().size(),1u);
    EXPECT_EQ(model.contributions()->parents()[0].node_count,8u);
  }
}
TEST(ExtendedSolidModel, ParentNodeMaterialAndUniqueCurveCapsRejectThenRetry) {
  Fixture f; f.Repeat44(); f.Repeat90();
  const auto domain = f.Domain();
  for (unsigned choice = 0; choice < 4; ++choice) {
    s::ModelLimits limits;
    if (choice == 0) limits.max_parents = 6;
    if (choice == 1) limits.max_nodes = domain.node_count()-1;
    if (choice == 2) limits.max_materials = 3;
    if (choice == 3) limits.max_curve_points = 8;
    s::Model model;
    EXPECT_EQ(model.Initialize(domain,f.Input(),limits).status,s::ModelStatus::ResourceLimit);
    Empty(model);
    limits = {}; limits.max_materials = 4; limits.max_curve_points = 9;
    ASSERT_TRUE(model.Initialize(domain,f.Input(),limits));
  }
  for (unsigned family = 0; family < 2; ++family) {
    auto input = f.Input();
    if (family == 0) input.solid18_law44 = {reinterpret_cast<const s::Input18Law44*>(1),SIZE_MAX};
    else input.solid18_law90 = {reinterpret_cast<const s::Input18Law90*>(1),SIZE_MAX};
    s::Model model;
    EXPECT_EQ(model.Initialize(domain,input).status,s::ModelStatus::ResourceLimit);
    Empty(model);
  }
}
TEST(ExtendedSolidModel, CompleteActualByteCapIncludesOwnedCurvesAndStaging) {
  Fixture f;
  const auto domain = f.Domain();
  s::Model measured;
  ASSERT_TRUE(measured.Initialize(domain,f.Input()));
  const auto bytes = measured.startup_payload_bytes();
  EXPECT_GT(bytes,measured.owned_payload_bytes());
  EXPECT_GT(measured.owned_payload_bytes(),domain.owned_payload_bytes());
  s::ModelLimits limits; limits.max_host_bytes = bytes-1;
  s::Model model;
  EXPECT_EQ(model.Initialize(domain,f.Input(),limits).status,s::ModelStatus::ResourceLimit);
  Empty(model);
  limits.max_host_bytes = bytes;
  ASSERT_TRUE(model.Initialize(domain,f.Input(),limits));
  EXPECT_EQ(model.startup_payload_bytes(),bytes);
  auto input = f.Input(); input.solid18_law90 = {reinterpret_cast<const s::Input18Law90*>(1),1};
  limits.max_host_bytes = 1;
  s::Model unopened;
  EXPECT_EQ(unopened.Initialize(domain,input,limits).status,s::ModelStatus::ResourceLimit);
  Empty(unopened);
  ASSERT_TRUE(unopened.Initialize(domain,f.Input()));
}
TEST(ExtendedSolidModel, InvalidBorrowedCurveAndDerivedPreparationNeverPublish) {
  Fixture f;
  const auto domain = f.Domain();
  const auto good = f.input44[0].material;
  for (unsigned choice = 0; choice < 5; ++choice) {
    f.input44[0].material = good;
    if (choice == 0) f.input44[0].material.curve.plastic_strain = nullptr;
    if (choice == 1) f.input44[0].material.curve.yield_stress_pa = reinterpret_cast<const double*>(1);
    if (choice == 2) f.input44[0].material.curve.count = UINT32_MAX;
    if (choice == 3) f.input44[0].material.shear_pa = std::nextafter(good.shear_pa,0.0);
    if (choice == 4) f.rear_y[2] = std::numeric_limits<double>::quiet_NaN();
    s::Model model;
    EXPECT_EQ(model.Initialize(domain,f.Input()).status,s::ModelStatus::InvalidInput);
    Empty(model);
    f.input44[0].material = good; f.rear_y[2] = 450e6;
    ASSERT_TRUE(model.Initialize(domain,f.Input()));
  }
  f.foam_y[2] = std::numeric_limits<double>::quiet_NaN();
  s::Model model;
  const auto report = model.Initialize(domain,f.Input());
  EXPECT_EQ(report.status,s::ModelStatus::InvalidInput);
  EXPECT_EQ(report.family,s::Family::Solid18Law90); Empty(model);
  f.foam_y[2] = 50e6;
  ASSERT_TRUE(model.Initialize(domain,f.Input()));
}
} // namespace extended_model_test
