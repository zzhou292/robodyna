#include "OriginalCatalog.h"

namespace qbat_catalog_test {

TEST(QbatCatalogOriginal, All4250QuadsAndOriginalTriangleShareOneMaterialAndSection) {
  SourceCatalog source;
  fe::ShellBatchBinding binding;
  ASSERT_EQ(binding.InitializeFormulations(source.geometry.Input(),fe::ShellHostBindingLimits::Vehicle()).status,
      fe::ShellBindingStatus::Success);
  Catalog catalog;
  EXPECT_EQ(catalog.InitializeFormulations(binding,source.Input()).status,Status::ResourceLimit);
  ASSERT_EQ(catalog.InitializeFormulationCatalog(binding,source.Input(),fe::ShellPlasticityCatalogLimits::Vehicle()).status,
      Status::Success);
  EXPECT_EQ(catalog.material_count(),1u);
  EXPECT_EQ(catalog.section_count(),1u);
  EXPECT_EQ(catalog.curve_count(),0u);
  EXPECT_EQ(catalog.parent_count(),4251u);
  EXPECT_EQ(binding.node_count(),4384u);
  EXPECT_EQ(binding.qeph_count(),0u);
  EXPECT_EQ(binding.t3_count(),1u);
  fe::ShellBatchFailureBinding failure;
  ASSERT_EQ(failure.Initialize(catalog,source.failures.data(),source.failures.size(),
      fe::ShellBatchFailureLimits::Vehicle()).status,Status::Success);
  ASSERT_EQ(fe::ValidateShellFormulationScope({&binding,&catalog,&failure}).status,Status::Success);
  for(std::size_t i=0;i<catalog.parent_count();++i) {
    const auto& expected=source.parents[i];
    const auto* p=catalog.parent(i);
    ASSERT_NE(p,nullptr);
    EXPECT_EQ(p->source_parent_id,expected.source_parent_id);
    EXPECT_EQ(p->source_part_id,2000524u);
    EXPECT_EQ(p->material_id,2000524u);
    EXPECT_EQ(p->section_id,2000524u);
    CheckMidlayer(catalog,p->family,p->family_index,p->family==Family::Qbat?
        fe::ShellSectionLaw::Law44QbatFourInPlane:fe::ShellSectionLaw::Law44Nip1);
    const auto* f=failure.parent(p->family,p->family_index);
    ASSERT_NE(f,nullptr);
    EXPECT_EQ(f->source.source_parent_id,p->source_parent_id);
    EXPECT_DOUBLE_EQ(f->constant.failure_strain,2.5);
  }
}

TEST(QbatCatalogOriginal, LastSourceAssignmentAndFailureRejectThenRetryAtExactBudgets) {
  SourceCatalog source;
  fe::ShellBatchBinding binding;
  ASSERT_EQ(binding.InitializeFormulations(source.geometry.Input(),fe::ShellHostBindingLimits::Vehicle()).status,
      fe::ShellBindingStatus::Success);
  auto limits=fe::ShellPlasticityCatalogLimits::Vehicle();
  Catalog reference;
  ASSERT_EQ(reference.InitializeFormulationCatalog(binding,source.Input(),limits).status,Status::Success);
  limits.max_owned_bytes=reference.host_bytes();
  limits.max_startup_scratch_bytes=reference.startup_scratch_bytes();
  Catalog retry;
  const auto old=Bytes(retry);
  source.parents.back().source_parent_id+=1;
  const auto rejected=retry.InitializeFormulationCatalog(binding,source.Input(),limits);
  EXPECT_EQ(rejected.status,Status::IdentityMismatch);
  EXPECT_EQ(rejected.entry,4250u);
  EXPECT_EQ(Bytes(retry),old);
  source.parents.back().source_parent_id-=1;
  ASSERT_EQ(retry.InitializeFormulationCatalog(binding,source.Input(),limits).status,Status::Success);
  EXPECT_TRUE(retry.SameScope(reference));
  fe::ShellBatchFailureBinding failure;
  source.failures.back().constant.failure_strain=0;
  const auto failure_old=Bytes(failure);
  const auto failure_bad=failure.Initialize(retry,source.failures.data(),4251,fe::ShellBatchFailureLimits::Vehicle());
  EXPECT_EQ(failure_bad.status,Status::InvalidMaterial);
  EXPECT_EQ(failure_bad.entry,4250u);
  EXPECT_EQ(Bytes(failure),failure_old);
  source.failures.back().constant.failure_strain=2.5;
  ASSERT_EQ(failure.Initialize(retry,source.failures.data(),4251,fe::ShellBatchFailureLimits::Vehicle()).status,Status::Success);
  std::printf("Original catalog: owned=%zu scratch=%zu failure=%zu binding=%zu\n",
      retry.host_bytes(),retry.startup_scratch_bytes(),failure.host_bytes(),binding.host_bytes());
}
} // namespace qbat_catalog_test
