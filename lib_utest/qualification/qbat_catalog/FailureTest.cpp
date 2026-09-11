#include "Fixture.h"

namespace qbat_catalog_test {
TEST(QbatCatalog, OldEntryPointsRemainClosedAndLateSourceMismatchRetries) {
  Fixture f;
  fe::ShellBatchBinding binding;
  ASSERT_EQ(binding.InitializeFormulations(f.Geometry()).status,fe::ShellBindingStatus::Success);
  Catalog catalog;
  EXPECT_EQ(catalog.Initialize(binding,f.Input()).status,Status::InvalidInput);
  EXPECT_EQ(catalog.InitializeSections(binding,f.Input()).status,Status::InvalidInput);
  EXPECT_EQ(catalog.InitializeCatalog(binding,f.Input(),{}).status,Status::InvalidInput);
  EXPECT_EQ(catalog.InitializeSectionCatalog(binding,f.Input(),{}).status,Status::InvalidInput);
  const auto old=Bytes(catalog);
  f.parents.back().source_parent_id+=1;
  const auto report=catalog.InitializeFormulations(binding,f.Input());
  EXPECT_EQ(report.status,Status::IdentityMismatch);
  EXPECT_EQ(report.entry,5u);
  EXPECT_EQ(report.family,Family::T3);
  EXPECT_EQ(Bytes(catalog),old);
  f.parents.back().source_parent_id-=1;
  ASSERT_EQ(catalog.InitializeFormulations(binding,f.Input()).status,Status::Success);
  EXPECT_EQ(catalog.InitializeFormulations(binding,f.Input()).status,Status::AlreadyInitialized);
}

TEST(QbatCatalog, QbatRequiresOneThicknessPointAndExactNativeA11) {
  Fixture f;
  fe::ShellBatchBinding binding;
  ASSERT_EQ(binding.InitializeFormulations(f.Geometry()).status,fe::ShellBindingStatus::Success);
  Catalog catalog;
  f.sections[2].through_thickness_points=3;
  f.sections[2].formulation=fe::ShellSectionFormulation::LayeredNip3;
  EXPECT_EQ(catalog.InitializeFormulations(binding,f.Input()).status,Status::InvalidSection);
  EXPECT_FALSE(catalog.prepared());
  f.sections[2]=MidlayerSection();
  f.materials[2].law=fe::ShellSectionLaw::Law44QbatFourInPlane;
  EXPECT_EQ(catalog.InitializeFormulations(binding,f.Input()).status,Status::InvalidMaterial);
  f.materials[2]=Midlayer();
  ASSERT_EQ(catalog.InitializeFormulations(binding,f.Input()).status,Status::Success);
  // A valid native reference with an explicitly different initial A11 must not
  // acquire this unchanged constitutive declaration by matching only E/rho/nu.
  f.geometry.b.reference.initial_a11_pa=std::nextafter(f.geometry.b.reference.initial_a11_pa,INFINITY);
  fe::ShellBatchBinding different;
  ASSERT_EQ(different.InitializeFormulations(f.Geometry()).status,fe::ShellBindingStatus::Success);
  Catalog rejected;
  const auto report=rejected.InitializeFormulations(different,f.Input());
  EXPECT_EQ(report.status,Status::IdentityMismatch);
  EXPECT_EQ(report.family,Family::Qbat);
  EXPECT_EQ(report.entry,0u);
}

TEST(QbatCatalog, PreflightRejectsCountsExtentsAndOneByteShortWithoutReadingPayload) {
  Fixture f;
  fe::ShellBatchBinding binding;
  ASSERT_EQ(binding.InitializeFormulations(f.Geometry()).status,fe::ShellBindingStatus::Success);
  Catalog valid;
  ASSERT_EQ(valid.InitializeFormulations(binding,f.Input()).status,Status::Success);
  auto limits=fe::ShellPlasticityCatalogLimits{};
  limits.max_owned_bytes=valid.host_bytes();
  limits.max_startup_scratch_bytes=valid.startup_scratch_bytes();
  for(unsigned fault=0;fault<4;++fault) {
    Catalog rejected;
    const auto old=Bytes(rejected);
    auto input=f.Input();
    auto cap=limits;
    Status expected=Status::ResourceLimit;
    if(fault==0) {
      input.parent_count=SIZE_MAX;
      input.parents=reinterpret_cast<const fe::ShellPlasticityParentInput*>(1);
    } else if(fault==1) {
      cap.max_owned_bytes-=1;
      input.parents=reinterpret_cast<const fe::ShellPlasticityParentInput*>(1);
    } else if(fault==2) {
      cap.max_startup_scratch_bytes-=1;
      input.parents=reinterpret_cast<const fe::ShellPlasticityParentInput*>(1);
    } else {
      input.parents=reinterpret_cast<const fe::ShellPlasticityParentInput*>(UINTPTR_MAX-7);
      expected=Status::InvalidInput;
    }
    EXPECT_EQ(rejected.InitializeFormulationCatalog(binding,input,cap).status,expected)<<fault;
    EXPECT_EQ(Bytes(rejected),old);
    ASSERT_EQ(rejected.InitializeFormulationCatalog(binding,f.Input(),limits).status,Status::Success);
  }
}

TEST(QbatCatalog, FourPointFailureDeclarationRejectsNoneTab1AndWrongScopeAtomically) {
  Fixture f;
  fe::ShellBatchBinding binding;
  ASSERT_EQ(binding.InitializeFormulations(f.Geometry()).status,fe::ShellBindingStatus::Success);
  Catalog catalog;
  ASSERT_EQ(catalog.InitializeFormulations(binding,f.Input()).status,Status::Success);
  for(unsigned fault=0;fault<4;++fault) {
    auto rows=f.failures;
    if(fault==0) { rows[0].policy=fe::ShellFailurePolicy::None; rows[0].constant={}; }
    if(fault==1) rows[0].policy=fe::ShellFailurePolicy::Tab1AnyPoint;
    if(fault==2) rows[0].tab1.table.failure_strain=.2;
    if(fault==3) rows.back().constant.failure_strain=std::numeric_limits<double>::infinity();
    fe::ShellBatchFailureBinding failure;
    const auto old=Bytes(failure);
    const auto report=failure.Initialize(catalog,rows.data(),rows.size());
    EXPECT_EQ(report.status,Status::InvalidMaterial);
    EXPECT_EQ(report.entry,fault==3?5u:0u);
    EXPECT_EQ(Bytes(failure),old);
    ASSERT_EQ(failure.Initialize(catalog,f.failures.data(),6).status,Status::Success);
    EXPECT_EQ(fe::ValidateShellFormulationScope({&binding,&catalog,&failure}).status,Status::Success);
    f.materials[2].linear.initial_yield_pa+=1;
    Catalog changed;
    ASSERT_EQ(changed.InitializeFormulations(binding,f.Input()).status,Status::Success);
    EXPECT_EQ(fe::ValidateShellFormulationScope({&binding,&changed,&failure}).status,Status::IdentityMismatch);
    f.materials[2].linear.initial_yield_pa-=1;
  }
  EXPECT_EQ(fe::ValidateShellFormulationScope({&binding,&catalog,nullptr}).status,Status::InvalidInput);
}
} // namespace qbat_catalog_test
