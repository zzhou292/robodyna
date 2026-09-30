#include "PlasticityBindingFixture.h"
#include "lib_src/elements/ShellBatchSectionBinding.h"

namespace plasticity_binding_test {
namespace {
using Law = fe::ShellSectionLaw;
using Form = fe::ShellSectionFormulation;
using Family = fe::ShellBindingFamily;
void OnePoint(Fixture& f) {
  f.sections[1].through_thickness_points = 1;
  f.sections[1].formulation = Form::OneThicknessPoint;
}
}

TEST(ShellSectionBinding, OneMaterialResolvesDistinctT3PointRolesWithoutDuplicatedParameters) {
  Fixture f;
  OnePoint(f);
  std::array<fe::ShellT3BindingInput, 2> triangles{{f.t3, f.t3}};
  triangles[1].source_parent_id = 703;
  for (unsigned n = 0; n < 3; ++n) {
    triangles[1].nodes[n] = 5 + n;
    triangles[1].reference.node_ids[n] = 105 + n;
    triangles[1].reference.position[n].z += .25;
  }
  fe::ShellBatchBinding geometry;
  ASSERT_EQ(geometry.Initialize({&f.qeph, triangles.data(), 1, 2, 8}).status,
            fe::ShellBindingStatus::Success);
  const fe::ShellPlasticitySectionInput sections[]{f.sections[0], f.sections[1], {59, 1./64, 3}};
  const fe::ShellPlasticityParentInput parents[]{f.parents[0], f.parents[1],
      {Family::T3, 1, 703, 83, 38, 59}};
  auto input = f.catalog();
  input.sections = sections;
  input.section_count = 3;
  input.parents = parents;
  input.parent_count = 3;
  Binding catalog;
  ASSERT_EQ(catalog.InitializeSections(geometry, input).status, Status::Success);
  EXPECT_EQ(catalog.material_count(), 2u);
  Law first = Law::Unspecified, last = Law::Unspecified;
  ASSERT_TRUE(catalog.Law(Family::T3, 0, &first));
  ASSERT_TRUE(catalog.Law(Family::T3, 1, &last));
  EXPECT_EQ(first, Law::Law44Nip1);
  EXPECT_EQ(last, Law::LayeredLaw44Nip3);
  fe::ShellSectionCounts counts;
  ASSERT_TRUE(catalog.Counts(Family::T3, &counts));
  EXPECT_EQ(counts.law44, 2u);
  EXPECT_EQ(counts.law44_nip1, 1u);
  EXPECT_EQ(counts.law1, 0u);
  fe::sections::PointParameters a, b;
  ASSERT_TRUE(catalog.Parameters(Family::T3, 0, &a));
  ASSERT_TRUE(catalog.Parameters(Family::T3, 1, &b));
  EXPECT_EQ(Bytes(a), Bytes(b));
  Binding copy(catalog);
  EXPECT_TRUE(copy.SameScope(catalog));
  ASSERT_TRUE(copy.Parameters(Family::T3, 0, &a));
  EXPECT_NE(a.curve.plastic_strain, b.curve.plastic_strain);
  EXPECT_EQ(a.curve.yield_stress_pa[2], b.curve.yield_stress_pa[2]);
}

TEST(ShellSectionBinding, OnePointRoleRequiresExplicitCountFamilyMaterialAndCenteredReference) {
  for (unsigned fault = 0; fault < 6; ++fault) {
    Fixture f;
    OnePoint(f);
    if (fault == 5) f.t3.reference.placement = fe::ShellReferencePlacement::TopReferencePlane;
    fe::ShellBatchBinding geometry;
    ASSERT_EQ(geometry.Initialize(f.collection()).status, fe::ShellBindingStatus::Success);
    switch (fault) {
      case 0: f.sections[1].formulation = Form::LayeredNip3; break;
      case 1: f.sections[1].through_thickness_points = 3; break;
      case 2: f.sections[1].formulation = static_cast<Form>(255); break;
      case 3:
        f.sections[0].formulation = Form::OneThicknessPoint;
        f.sections[0].through_thickness_points = 1;
        break;
      case 4: f.materials[1].law = Law::Law44Nip1; break;
      case 5: break;
    }
    Binding rejected;
    const auto before = Bytes(rejected);
    const auto report = rejected.InitializeSections(geometry, f.catalog());
    EXPECT_EQ(report.status, fault == 4 ? Status::InvalidMaterial : Status::InvalidSection) << fault;
    EXPECT_EQ(Bytes(rejected), before) << fault;
    Fixture good;
    OnePoint(good);
    fe::ShellBatchBinding good_geometry;
    ASSERT_EQ(good_geometry.Initialize(good.collection()).status, fe::ShellBindingStatus::Success);
    EXPECT_EQ(rejected.InitializeSections(good_geometry, good.catalog()).status, Status::Success);
  }
}

TEST(ShellSectionBinding, OnePointRoleIsCompleteIdentityAndLegacyAdmissionStaysClosed) {
  Fixture f;
  fe::ShellBatchBinding geometry;
  ASSERT_EQ(geometry.Initialize(f.collection()).status, fe::ShellBindingStatus::Success);
  Binding old;
  ASSERT_EQ(old.InitializeSections(geometry, f.catalog()).status, Status::Success);
  OnePoint(f);
  Binding one;
  ASSERT_EQ(one.InitializeSections(geometry, f.catalog()).status, Status::Success);
  EXPECT_FALSE(old.SameScope(one));
  Binding legacy;
  const auto before = Bytes(legacy);
  EXPECT_EQ(legacy.Initialize(geometry, f.catalog()).status, Status::InvalidSection);
  EXPECT_EQ(legacy.InitializeCatalog(geometry, f.catalog(), {}).status, Status::InvalidSection);
  EXPECT_EQ(Bytes(legacy), before);
  fe::ShellPlasticityCatalogLimits limits;
  limits.max_owned_bytes = one.host_bytes();
  limits.max_startup_scratch_bytes = one.startup_scratch_bytes();
  auto low = limits;
  --low.max_owned_bytes;
  EXPECT_EQ(legacy.InitializeSectionCatalog(geometry, f.catalog(), low).status, Status::ResourceLimit);
  EXPECT_EQ(Bytes(legacy), before);
  EXPECT_EQ(legacy.InitializeSectionCatalog(geometry, f.catalog(), limits).status, Status::Success);
}
} // namespace plasticity_binding_test
