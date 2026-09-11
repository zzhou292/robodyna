#include "HostShellFixture.h"
#include "AllocationFailure.h"
#include <memory>
#include <type_traits>

namespace host_shell_test {
using Status=fe::ShellPlasticityBindingStatus;
static_assert(std::is_nothrow_copy_constructible_v<Catalog> && std::is_nothrow_move_constructible_v<Catalog>);
static_assert(!std::is_copy_assignable_v<Catalog> && !std::is_move_assignable_v<Catalog>);
TEST(HostShellCatalog, CompleteSixMaterialTwoCurveParentMapping) {
  Fixture f; Geometry native; Catalog catalog;
  ASSERT_EQ(native.Initialize(f.geometry(),{}).status,fe::ShellBindingStatus::Success);
  ASSERT_EQ(catalog.Initialize(native,f.catalog()).status,Status::Success);
  EXPECT_EQ(catalog.material_count(),6u); EXPECT_EQ(catalog.section_count(),6u);
  EXPECT_EQ(catalog.curve_count(),2u); EXPECT_EQ(catalog.curve_point_count(),63u);
  EXPECT_EQ(catalog.parent_count(),915u); EXPECT_TRUE(catalog.Matches(native));
  EXPECT_EQ(catalog.inventory().words().size(),25069u);
  std::array<std::size_t,6> observed{};
  for(std::size_t i=0;i<f.parents.size();++i) {
    const auto& expected=f.parents[i]; const auto* p=catalog.parent(i); ASSERT_NE(p,nullptr);
    EXPECT_EQ(p->source_parent_id,expected.source_parent_id); EXPECT_EQ(p->family_index,expected.family_index);
    EXPECT_EQ(p->material_id,expected.material_id); EXPECT_EQ(p->section_id,expected.section_id);
    const auto part=p->material_id-100; ASSERT_LT(part,6u); ++observed[part];
    fe::sections::PointParameters coefficients;
    ASSERT_TRUE(catalog.Parameters(p->family,p->family_index,&coefficients));
    EXPECT_EQ(coefficients.young_pa,200e9); EXPECT_EQ(coefficients.density_kg_m3,7890);
    EXPECT_EQ(coefficients.curve.count,part==2?17u:46u);
    EXPECT_EQ(coefficients.curve.yield_stress_pa[0],part==2?180e6:270e6);
  }
  EXPECT_EQ(observed,(std::array<std::size_t,6>{73,138,548,94,33,29}));
  EXPECT_EQ(catalog.parent(915),nullptr);
  RecordProperty("host_bytes",std::to_string(catalog.host_bytes()));
  EXPECT_GT(catalog.host_bytes(),catalog.inventory().backing_bytes()+sizeof(Catalog));
  EXPECT_LT(catalog.host_bytes(),512u*1024);
}
TEST(HostShellCatalog, IndependentByteAndDeclarationCapsPrecedeBorrowedAccess) {
  Fixture f; Geometry native; Catalog good;
  ASSERT_EQ(native.Initialize(f.geometry(),{}).status,fe::ShellBindingStatus::Success);
  ASSERT_EQ(good.Initialize(native,f.catalog()).status,Status::Success);
  Catalog b; const auto old=Bytes(b); auto input=f.catalog();
  input.curves=reinterpret_cast<const fe::ShellPlasticityCurveInput*>(1);
  input.materials=reinterpret_cast<const fe::ShellPlasticityMaterialInput*>(1);
  input.sections=reinterpret_cast<const fe::ShellPlasticitySectionInput*>(1);
  input.parents=reinterpret_cast<const fe::ShellPlasticityParentInput*>(1);
  fe::ShellHostBindingLimits limits; limits.max_owned_bytes=good.host_bytes()-1;
  EXPECT_EQ(b.Initialize(native,input,limits).status,Status::ResourceLimit); EXPECT_EQ(Bytes(b),old);
  limits={}; limits.max_parents=914;
  EXPECT_EQ(b.Initialize(native,input,limits).status,Status::ResourceLimit); EXPECT_EQ(Bytes(b),old);
  input.material_count=std::numeric_limits<std::size_t>::max();
  EXPECT_EQ(b.Initialize(native,input).status,Status::ResourceLimit); EXPECT_EQ(Bytes(b),old);
  limits={}; limits.max_owned_bytes=good.host_bytes();
  ASSERT_EQ(b.Initialize(native,f.catalog(),limits).status,Status::Success);
  EXPECT_EQ(b.host_bytes(),limits.max_owned_bytes); EXPECT_TRUE(b.SameScope(good));
}
TEST(HostShellCatalog, LateInvalidDeclarationsAndPoolCapPreserveBytesThenRetry) {
  Fixture valid; Geometry native;
  ASSERT_EQ(native.Initialize(valid.geometry(),{}).status,fe::ShellBindingStatus::Success);
  for(unsigned failure=0;failure<5;++failure) {
    Fixture f; Catalog b; const auto old=Bytes(b);
    if(failure==0) f.y1.back()=std::numeric_limits<double>::quiet_NaN();
    if(failure==1) f.materials.back().young_pa=-1;
    if(failure==2) { f.parents.back().material_id=999; f.parents.back().source_part_id=999; }
    if(failure==3) f.parents.back().family_index=TCount;
    if(failure==4) f.curves.back().curve.count=fe::MaxShellPlasticityCurvePoints-46+1;
    const Status expected[]{Status::InvalidCurve,Status::InvalidMaterial,Status::InvalidParent,
                            Status::InvalidParent,Status::ResourceLimit};
    const auto report=b.Initialize(native,f.catalog());
    EXPECT_EQ(report.status,expected[failure])<<failure; EXPECT_EQ(Bytes(b),old);
    EXPECT_EQ(b.Initialize(native,valid.catalog()).status,Status::Success);
  }
}
TEST(HostShellCatalog, EveryExpandedCatalogAllocationFailureIsAtomic) {
  Fixture f; Geometry native;
  ASSERT_EQ(native.Initialize(f.geometry(),{}).status,fe::ShellBindingStatus::Success);
  // Parent map array/control and Q family lookup array/control. Shared complete
  // inventory copy and fixed curve/material/section pools require no allocation.
  for(std::ptrdiff_t after=0;after<4;++after) {
    Catalog b; const auto old=Bytes(b); fe::ShellPlasticityBindingReport report;
    { AllocationFailure fail(after); report=b.Initialize(native,f.catalog()); }
    EXPECT_EQ(report.status,Status::ResourceLimit)<<after; EXPECT_EQ(Bytes(b),old);
    EXPECT_EQ(b.Initialize(native,f.catalog()).status,Status::Success);
  }
}
TEST(HostShellCatalog, SharedMappingsAndIndependentCurveViewsSurviveAllInputLifetimes) {
  auto f=std::make_unique<Fixture>(); auto native=std::make_unique<Geometry>();
  auto original=std::make_unique<Catalog>();
  ASSERT_EQ(native->Initialize(f->geometry(),{}).status,fe::ShellBindingStatus::Success);
  ASSERT_EQ(original->Initialize(*native,f->catalog()).status,Status::Success);
  Catalog copy(*original),moved(std::move(copy));
  f.reset(); original.reset(); native.reset();
  EXPECT_TRUE(copy.prepared()); EXPECT_TRUE(copy.SameScope(moved));
  fe::sections::PointParameters p;
  ASSERT_TRUE(moved.Parameters(fe::ShellBindingFamily::T3,110,&p));
  EXPECT_EQ(p.curve.yield_stress_pa[45],315e6);
  bool copied=false;
  { AllocationFailure fail;
    Catalog a(copy),b(std::move(a)); copied=a.SameScope(b)&&b.SameScope(moved);
  }
  EXPECT_TRUE(copied);
  Fixture changed; Geometry b; Catalog later;
  ASSERT_EQ(b.Initialize(changed.geometry(),{}).status,fe::ShellBindingStatus::Success);
  changed.y0.back()+=1.;
  ASSERT_EQ(later.Initialize(b,changed.catalog()).status,Status::Success);
  EXPECT_FALSE(moved.SameScope(later)); // Late curve point cannot hide behind shared inventory.
}
TEST(HostShellCatalog, LegacyCatalogStillInitializesAndCopiesWithoutAllocation) {
  plasticity_binding_test::Fixture f; Geometry native; Catalog catalog;
  ASSERT_EQ(native.Initialize(f.collection()).status,fe::ShellBindingStatus::Success);
  fe::ShellPlasticityBindingReport report; bool copied=false;
  { AllocationFailure fail;
    report=catalog.Initialize(native,f.catalog()); Catalog a(catalog),b(std::move(a));
    copied=a.SameScope(b)&&b.SameScope(catalog);
  }
  ASSERT_EQ(report.status,Status::Success); EXPECT_TRUE(copied);
  EXPECT_EQ(catalog.host_bytes(),sizeof(Catalog));
}
TEST(HostShellCatalog, ExpandedDeclarationTablesAndT3MapAreCoveredAndBounded) {
  constexpr std::size_t count=129;
  plasticity_binding_test::Fixture pair;
  std::vector<fe::ShellT3BindingInput> t(count,pair.t3);
  std::vector<fe::ShellPlasticityCurveInput> curves(count);
  std::vector<fe::ShellPlasticityMaterialInput> materials(count);
  std::vector<fe::ShellPlasticitySectionInput> sections(count);
  std::vector<fe::ShellPlasticityParentInput> parents(count);
  for(std::size_t i=0;i<count;++i) {
    t[i].nodes={0,1,2}; t[i].source_parent_id=1000+i;
    curves[i]={2000+i,{pair.x,pair.yt,2}};
    materials[i]={3000+i,2000+i,3e6,.25,768,{}};
    sections[i]={4000+i,1./64,3};
    parents[i]={fe::ShellBindingFamily::T3,i,1000+i,5000+i,3000+i,4000+i};
  }
  Geometry native;
  ASSERT_EQ(native.Initialize({nullptr,t.data(),0,count,3},{}).status,fe::ShellBindingStatus::Success);
  fe::ShellBatchPlasticityBindingInput input{curves.data(),materials.data(),sections.data(),parents.data(),
                                           count,count,count,count};
  Catalog good; ASSERT_EQ(good.Initialize(native,input).status,Status::Success);
  fe::sections::PointParameters p; ASSERT_TRUE(good.Parameters(fe::ShellBindingFamily::T3,128,&p));
  EXPECT_EQ(p.curve.yield_stress_pa[1],6800); EXPECT_EQ(good.curve_point_count(),258u);
  fe::ShellHostBindingLimits limits; limits.max_owned_bytes=good.host_bytes()-1;
  Catalog capped; const auto old=Bytes(capped);
  EXPECT_EQ(capped.Initialize(native,input,limits).status,Status::ResourceLimit); EXPECT_EQ(Bytes(capped),old);
  // Every expanded declaration array and the T3 lookup owns array/control allocations.
  for(std::ptrdiff_t after=0;after<10;++after) {
    Catalog b; const auto before=Bytes(b); fe::ShellPlasticityBindingReport report;
    { AllocationFailure fail(after); report=b.Initialize(native,input); }
    EXPECT_EQ(report.status,Status::ResourceLimit)<<after; EXPECT_EQ(Bytes(b),before);
  }
}
} // namespace host_shell_test
