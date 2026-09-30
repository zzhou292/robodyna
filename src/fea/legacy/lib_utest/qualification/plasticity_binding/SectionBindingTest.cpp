#include "PlasticityBindingFixture.h"
#include "lib_src/elements/ShellBatchSectionBinding.h"
#include <limits>
#include <memory>
#include <utility>

namespace plasticity_binding_test {
namespace {
using Law=fe::ShellSectionLaw;
using Family=fe::ShellBindingFamily;
using SectionBinding=fe::ShellBatchSectionBinding;
void Elastic(fe::ShellSectionMaterialInput& m) {
  m.law=Law::LayeredLaw1Nip3;m.curve_id=0;m.rate={};
  m.hardening=tl::material::ShellPlasticityHardeningKind::Tabulated;m.linear={};
}
fe::ShellBatchSectionBindingInput Mixed(Fixture& f) {
  Elastic(f.materials[0]);auto in=f.catalog();in.curves=f.curves.data()+1;in.curve_count=1;return in;
}
}

TEST(ShellSectionBinding, ExplicitModeOwnsCompleteMixedSourceAndTypedAvailability) {
  std::unique_ptr<SectionBinding> retained;
  {
    Fixture f;auto in=Mixed(f);fe::ShellBatchBinding native;
    ASSERT_EQ(native.Initialize(f.collection()).status,fe::ShellBindingStatus::Success);
    SectionBinding binding;ASSERT_EQ(binding.InitializeSections(native,in).status,Status::Success);
    EXPECT_TRUE(binding.heterogeneous_sections());EXPECT_TRUE(binding.Matches(native));
    EXPECT_EQ(binding.parent(0)->source_parent_id,702u);EXPECT_EQ(binding.parent(1)->source_parent_id,701u);
    SectionBinding copy(binding);retained=std::make_unique<SectionBinding>(std::move(copy));
    EXPECT_TRUE(binding.SameScope(*retained));
    f.materials[0].young_pa=1;f.yt[0]=-1; // Borrowed declarations and curves may disappear.
  }
  tl::material::ShellElasticLaw1PointParameters elastic,expected;
  ASSERT_TRUE(retained->ElasticParameters(Family::Qeph,0,&elastic));
  ASSERT_TRUE(tl::material::PrepareShellElasticLaw1Point(2e6,.3,1024,expected));
  EXPECT_EQ(Bytes(elastic),Bytes(expected));
  fe::sections::PointParameters plastic;ASSERT_TRUE(retained->Parameters(Family::T3,0,&plastic));
  EXPECT_EQ(plastic.curve.yield_stress_pa[0],5400);EXPECT_TRUE(plastic.rate.enabled);
  const auto old_elastic=Bytes(elastic);const auto old_plastic=Bytes(plastic);
  EXPECT_FALSE(retained->ElasticParameters(Family::T3,0,&elastic));
  EXPECT_FALSE(retained->Parameters(Family::Qeph,0,&plastic));
  EXPECT_EQ(Bytes(elastic),old_elastic);EXPECT_EQ(Bytes(plastic),old_plastic);
  for(auto family:{Family::Qeph,Family::T3}) {
    Law law=Law::Unspecified;fe::ShellSectionCounts count;
    ASSERT_TRUE(retained->Law(family,0,&law));ASSERT_TRUE(retained->Counts(family,&count));
    EXPECT_EQ(law,family==Family::Qeph?Law::LayeredLaw1Nip3:Law::LayeredLaw44Nip3);
    EXPECT_EQ(count.law1,family==Family::Qeph?1u:0u);EXPECT_EQ(count.law44,family==Family::T3?1u:0u);
    EXPECT_FALSE(retained->Law(family,1,&law));EXPECT_EQ(law,family==Family::Qeph?Law::LayeredLaw1Nip3:Law::LayeredLaw44Nip3);
  }
  fe::ShellSectionCounts counts{17,23};EXPECT_FALSE(retained->Counts(Family::None,&counts));
  EXPECT_EQ(counts.law44,17u);EXPECT_EQ(counts.law1,23u);
}

TEST(ShellSectionBinding, AnalyticAndElasticNoCurveCatalogsRetainExactModeAndWholeScope) {
  Fixture f;Elastic(f.materials[1]);f.materials[0].curve_id=0;
  f.materials[0].hardening=tl::material::ShellPlasticityHardeningKind::LinearLaw44;
  f.materials[0].linear={2700,20000};f.materials[0].rate={true,8000,8,10000};
  auto in=f.catalog();in.curve_count=0;in.curves=nullptr;
  fe::ShellBatchBinding native;ASSERT_EQ(native.Initialize(f.collection()).status,fe::ShellBindingStatus::Success);
  SectionBinding a;ASSERT_EQ(a.InitializeSectionCatalog(native,in,{}).status,Status::Success);
  fe::sections::PointParameters p;ASSERT_TRUE(a.Parameters(Family::Qeph,0,&p));
  EXPECT_EQ(p.curve.count,0u);EXPECT_EQ(p.curve.plastic_strain,nullptr);EXPECT_GT(p.plastic_hardening_pa,20000);
  const auto saved=f.materials[0];Elastic(f.materials[0]);SectionBinding b;
  ASSERT_EQ(b.InitializeSections(native,in).status,Status::Success);EXPECT_FALSE(a.SameScope(b));
  fe::ShellSectionCounts count;ASSERT_TRUE(b.Counts(Family::Qeph,&count));EXPECT_EQ(count.law1,1u);
  f.materials[0]=saved;f.parents[0].source_part_id=83;SectionBinding changed;
  ASSERT_EQ(changed.InitializeSections(native,in).status,Status::Success);EXPECT_FALSE(a.SameScope(changed));
  Fixture tables;fe::ShellBatchBinding native_tables;
  ASSERT_EQ(native_tables.Initialize(tables.collection()).status,fe::ShellBindingStatus::Success);
  SectionBinding legacy,explicit_mode;
  ASSERT_EQ(legacy.Initialize(native_tables,tables.catalog()).status,Status::Success);
  ASSERT_EQ(explicit_mode.InitializeSections(native_tables,tables.catalog()).status,Status::Success);
  EXPECT_FALSE(legacy.SameScope(explicit_mode));
  EXPECT_FALSE(legacy.heterogeneous_sections());
}

TEST(ShellSectionBinding, MixedLawsWithinFamilyUseNativeParentIndexAndCompleteSourceOrder) {
  Fixture f;auto input=Mixed(f);std::array<fe::ShellQephBindingInput,2> q{{f.qeph,f.qeph}};
  q[1].source_parent_id=703;auto& second=q[1].reference;
  second.young_modulus=3e6;second.poisson_ratio=.25;second.density=768;second.thickness=1./64;
  fe::ShellBatchBinding native;
  ASSERT_EQ(native.Initialize({q.data(),&f.t3,2,1,5}).status,fe::ShellBindingStatus::Success);
  const fe::ShellSectionParentInput parents[]{f.parents[0],
    {Family::Qeph,1,703,83,38,58},f.parents[1]};
  input.parents=parents;input.parent_count=3;SectionBinding catalog;
  ASSERT_EQ(catalog.InitializeSections(native,input).status,Status::Success);
  Law first=Law::Unspecified,last=Law::Unspecified;fe::ShellSectionCounts count;
  ASSERT_TRUE(catalog.Law(Family::Qeph,0,&first));ASSERT_TRUE(catalog.Law(Family::Qeph,1,&last));
  ASSERT_TRUE(catalog.Counts(Family::Qeph,&count));EXPECT_EQ(count.law1,1u);EXPECT_EQ(count.law44,1u);
  EXPECT_EQ(first,Law::LayeredLaw1Nip3);EXPECT_EQ(last,Law::LayeredLaw44Nip3);
  EXPECT_EQ(catalog.parent(1)->source_parent_id,703u);
  fe::sections::PointParameters plastic;tl::material::ShellElasticLaw1PointParameters elastic;
  EXPECT_FALSE(catalog.Parameters(Family::Qeph,0,&plastic));
  ASSERT_TRUE(catalog.Parameters(Family::Qeph,1,&plastic));EXPECT_EQ(plastic.curve.yield_stress_pa[0],5400);
  ASSERT_TRUE(catalog.ElasticParameters(Family::Qeph,0,&elastic));EXPECT_EQ(elastic.young_pa,2e6);
}

TEST(ShellSectionBinding, InvalidLateElasticDeclarationsAndLegacyModeRejectAtomicallyThenRetry) {
  for(unsigned fault=0;fault<10;++fault) {
    Fixture f;Elastic(f.materials[1]);auto in=f.catalog();in.curve_count=1;
    fe::ShellBatchBinding native;ASSERT_EQ(native.Initialize(f.collection()).status,fe::ShellBindingStatus::Success);
    const auto clean=f.materials[1];
    switch(fault) {
      case 0:f.materials[1].curve_id=47;break;
      case 1:f.materials[1].rate.enabled=true;break;
      case 2:f.materials[1].rate.cutoff_hz=10000;break;
      case 3:f.materials[1].linear.initial_yield_pa=1;break;
      case 4:f.materials[1].hardening=tl::material::ShellPlasticityHardeningKind::LinearLaw44;break;
      case 5:f.materials[1].young_pa=std::numeric_limits<double>::infinity();break;
      case 6:f.materials[1].law=Law::Unspecified;break;
      case 7:f.materials[1].rate.cowper_symonds_c_per_s=-0.;break;
      case 8:f.materials[1].density_kg_m3=769;break;
      case 9:f.materials[1].poisson_ratio=.26;break;
    }
    SectionBinding binding;const auto before=Bytes(binding);
    const auto report=binding.InitializeSections(native,in);
    EXPECT_EQ(report.status,fault<8?Status::InvalidMaterial:Status::IdentityMismatch)<<fault;
    EXPECT_EQ(Bytes(binding),before)<<fault;
    f.materials[1]=clean;ASSERT_EQ(binding.InitializeSections(native,in).status,Status::Success);
    SectionBinding legacy;const auto virgin=Bytes(legacy);
    EXPECT_EQ(legacy.Initialize(native,in).status,Status::InvalidMaterial);EXPECT_EQ(Bytes(legacy),virgin);
    EXPECT_EQ(legacy.InitializeCatalog(native,in,{}).status,Status::InvalidMaterial);EXPECT_EQ(Bytes(legacy),virgin);
  }
}

TEST(ShellSectionBinding, ExactOwnedScratchBudgetsAndCountPreflightPreserveRetry) {
  Fixture f;auto in=Mixed(f);fe::ShellBatchBinding native;
  ASSERT_EQ(native.Initialize(f.collection()).status,fe::ShellBindingStatus::Success);
  SectionBinding ready;ASSERT_EQ(ready.InitializeSections(native,in).status,Status::Success);
  auto limits=fe::ShellSectionCatalogLimits{};limits.max_owned_bytes=ready.host_bytes();
  limits.max_startup_scratch_bytes=ready.startup_scratch_bytes();
  for(bool scratch:{false,true}) {
    auto low=limits;if(scratch)--low.max_startup_scratch_bytes;else --low.max_owned_bytes;
    SectionBinding retry;const auto before=Bytes(retry);
    EXPECT_EQ(retry.InitializeSectionCatalog(native,in,low).status,Status::ResourceLimit);
    EXPECT_EQ(Bytes(retry),before);EXPECT_EQ(retry.InitializeSectionCatalog(native,in,limits).status,Status::Success);
  }
  SectionBinding preflight;const auto before=Bytes(preflight);
  auto invalid=in;invalid.materials=reinterpret_cast<const fe::ShellSectionMaterialInput*>(1);
  invalid.material_count=std::numeric_limits<std::size_t>::max();
  EXPECT_EQ(preflight.InitializeSectionCatalog(native,invalid,limits).status,Status::ResourceLimit);
  EXPECT_EQ(Bytes(preflight),before);
  EXPECT_EQ(preflight.InitializeSections(native,invalid,fe::ShellHostBindingLimits::Vehicle()).status,Status::ResourceLimit);
  EXPECT_EQ(Bytes(preflight),before);EXPECT_EQ(preflight.InitializeSections(native,in).status,Status::Success);
}
} // namespace plasticity_binding_test
