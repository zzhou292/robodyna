// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
namespace extended_model_test {
TEST(ExtendedSolidModel, CrossLawMidsRejectBeforePublicationAndRestoreValidRetry) {
  for (unsigned choice = 0; choice < 5; ++choice) {
    Fixture f;
    const auto good44 = f.input44[0].reference;
    const auto good90 = f.input90[0].reference;
    if (choice < 2) {
      auto input = good44.input(); input.source_material_id = choice == 0 ? 20 : 30;
      ASSERT_EQ(rear::InitializeReference(input,f.input44[0].reference),fe::solid18::Status::Success);
    } else {
      auto input = good90.input(); input.source_material_id = choice == 2 ? 20 : choice == 3 ? 30 : 44;
      ASSERT_EQ(foam::InitializeReference90(input,f.input90[0].reference),fe::solid18::Status::Success);
    }
    s::Model model;
    const auto report = model.Initialize(f.Domain(),f.Input());
    EXPECT_EQ(report.status,s::ModelStatus::MaterialMismatch);
    EXPECT_EQ(report.family,choice < 2 ? s::Family::Solid18Law44 : s::Family::Solid18Law90);
    EXPECT_EQ(report.parent,0u); Empty(model);
    f.input44[0].reference = good44; f.input90[0].reference = good90;
    ASSERT_TRUE(model.Initialize(f.Domain(),f.Input()));
  }
}
TEST(ExtendedSolidModel, RepeatedMidChangedCurveAndPreparedValuesRejectExactly) {
  Fixture f; f.Repeat44(); f.Repeat90();
  const auto good44 = f.input44[1].material;
  const auto good90 = f.input90[1].material;
  double changed[]{270e6,350e6,std::nextafter(450e6,1e9)};
  ASSERT_EQ(law44::Prepare(good44.material,{f.rear_x,changed,3},f.input44[1].material),law44::Status::Ok);
  s::Model model;
  auto report = model.Initialize(f.Domain(),f.Input());
  EXPECT_EQ(report.status,s::ModelStatus::MaterialMismatch);
  EXPECT_EQ(report.family,s::Family::Solid18Law44); EXPECT_EQ(report.parent,1u);
  f.input44[1].material = good44;
  auto foam_input = f.foam_input; foam_input.contact_modulus_pa *= 2;
  ASSERT_EQ(law90::PrepareSI(foam_input,{f.foam_x,f.foam_y,3},f.input90[1].material),law90::Status::Ok);
  report = model.Initialize(f.Domain(),f.Input());
  EXPECT_EQ(report.status,s::ModelStatus::MaterialMismatch);
  EXPECT_EQ(report.family,s::Family::Solid18Law90); EXPECT_EQ(report.parent,1u);
  Empty(model); f.input90[1].material = good90;
  ASSERT_TRUE(model.Initialize(f.Domain(),f.Input()));
}
TEST(ExtendedSolidModel, LastFamilySourceAndDensityFailuresLeaveEmptyHandleForRetry) {
  Fixture f;
  const auto domain = f.Domain();
  const auto good = f.input90[0];
  auto input = good.reference.input(); input.source_element_id = 101;
  ASSERT_EQ(foam::InitializeReference90(input,f.input90[0].reference),fe::solid18::Status::Success);
  s::Model model;
  auto report = model.Initialize(domain,f.Input());
  EXPECT_EQ(report.status,s::ModelStatus::DuplicateIdentity);
  EXPECT_EQ(report.family,s::Family::Solid18Law90); EXPECT_EQ(report.parent,0u);
  input = good.reference.input(); input.position_m[7].z = std::nextafter(input.position_m[7].z,1.0);
  ASSERT_EQ(foam::InitializeReference90(input,f.input90[0].reference),fe::solid18::Status::Success);
  EXPECT_EQ(model.Initialize(domain,f.Input()).status,s::ModelStatus::SourceMismatch);
  input = good.reference.input(); input.source_node_id[7] += 900000;
  ASSERT_EQ(foam::InitializeReference90(input,f.input90[0].reference),fe::solid18::Status::Success);
  EXPECT_EQ(model.Initialize(domain,f.Input()).status,s::ModelStatus::SourceMismatch);
  f.input90[0] = good;
  f.foam_input.reference_density_kg_m3 = 700; f.PrepareFoam();
  report = model.Initialize(domain,f.Input());
  EXPECT_EQ(report.status,s::ModelStatus::InvalidInput);
  EXPECT_EQ(report.family,s::Family::Solid18Law90);
  Empty(model); f.input90[0] = good;
  ASSERT_TRUE(model.Initialize(domain,f.Input()));
  const auto* parents = model.solid18_law90().data();
  EXPECT_EQ(model.Initialize(domain,f.Input()).status,s::ModelStatus::AlreadyInitialized);
  EXPECT_EQ(model.solid18_law90().data(),parents);
}
} // namespace extended_model_test
