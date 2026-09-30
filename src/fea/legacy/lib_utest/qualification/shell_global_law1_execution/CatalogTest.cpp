// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../plasticity_binding/PlasticityBindingFixture.h"
#include "../qbat_catalog/Fixture.h"
#include "lib_src/elements/ShellBatchLayeredSection.h"
#include "lib_src/elements/ShellBatchFailureBinding.h"
#include <limits>
#include <memory>
namespace global_law1_execution_catalog_test {
namespace fe=tl::fea;using Status=fe::ShellPlasticityBindingStatus;using Catalog=fe::ShellBatchPlasticityBinding;
using Family=fe::ShellBindingFamily;using Law=fe::ShellSectionLaw;using Policy=fe::ShellParentExecutionPolicy;
using Thickness=fe::ShellLaw1Thickness;
using plasticity_binding_test::Bytes;using qbat_binding_test::Bits;
fe::ShellParentExecution Global() {return {Policy::GlobalLaw1Npt0,{Thickness::Accepted,.001}};}
struct Source:plasticity_binding_test::Fixture {
  bool same_part=false;
  explicit Source(bool same=false):same_part(same) {
    auto& elastic=materials[0];elastic.law=Law::LayeredLaw1Nip3;elastic.curve_id=0;elastic.rate={};
    // One real shared SID remains raw LayeredNip3/3, without a fabricated
    // section per resolved execution law. Material/PID distinction is explicit.
    t3.reference.thickness=sections[0].thickness_m;
    parents[0].section_id=parents[1].section_id=sections[0].section_id;
    parents[1].execution=Global();
    if(same) {
      t3.reference.young_modulus=qeph.reference.young_modulus;
      t3.reference.poisson_ratio=qeph.reference.poisson_ratio;t3.reference.density=qeph.reference.density;
      parents[0].material_id=parents[1].material_id;parents[0].source_part_id=parents[1].source_part_id;
      parents[0].execution=parents[1].execution;
    }
  }
  fe::ShellBatchPlasticityBindingInput Input() const {
    return {same_part?nullptr:curves.data()+1,materials.data(),sections.data(),parents.data(),
      same_part?0u:1u,same_part?1u:2u,1,2};
  }
};
void SameProfile(const fe::ShellGlobalLaw1Profile& a,const fe::ShellGlobalLaw1Profile& b) {
  EXPECT_EQ(a.thickness,b.thickness);EXPECT_EQ(Bits(a.coefficient_working_length_m),Bits(b.coefficient_working_length_m));
}
TEST(GlobalLaw1ExecutionCatalog, SharedRawSectionKeepsIdsAndResolvesOnlyExplicitGlobalParents) {
  Source source;fe::ShellBatchBinding shells;ASSERT_EQ(shells.Initialize(source.collection()).status,fe::ShellBindingStatus::Success);
  Catalog catalog;ASSERT_EQ(catalog.InitializeExecutionCatalog(shells,source.Input()).status,Status::Success);
  ASSERT_EQ(catalog.parent_count(),2u);EXPECT_EQ(catalog.section_count(),1u);
  EXPECT_EQ(catalog.parent(0)->source_parent_id,702u);EXPECT_EQ(catalog.parent(1)->source_parent_id,701u);
  EXPECT_EQ(catalog.parent(0)->section_id,catalog.parent(1)->section_id);
  EXPECT_EQ(source.sections[0].through_thickness_points,3u);
  EXPECT_EQ(source.sections[0].formulation,fe::ShellSectionFormulation::LayeredNip3);
  EXPECT_EQ(source.materials[0].law,Law::LayeredLaw1Nip3);
  Law q=Law::Unspecified,t=q;ASSERT_TRUE(catalog.Law(Family::Qeph,0,&q));ASSERT_TRUE(catalog.Law(Family::T3,0,&t));
  EXPECT_EQ(q,Law::GlobalLaw1Npt0);EXPECT_EQ(t,Law::LayeredLaw44Nip3);
  unsigned qp=99,tp=99;ASSERT_TRUE(catalog.MaterialPointCount(Family::Qeph,0,&qp));ASSERT_TRUE(catalog.MaterialPointCount(Family::T3,0,&tp));
  EXPECT_EQ(qp,0u);EXPECT_EQ(tp,3u);
  fe::ShellSectionCounts counts;ASSERT_TRUE(catalog.Counts(Family::Qeph,&counts));
  EXPECT_EQ(counts.law1,1u);EXPECT_EQ(counts.law1_global_npt0,1u);EXPECT_EQ(counts.law44,0u);EXPECT_EQ(counts.rigid_skin,0u);
  fe::ShellGlobalLaw1Profile profile;ASSERT_TRUE(catalog.GlobalLaw1Profile(Family::Qeph,0,&profile));SameProfile(profile,Global().global_law1);
  const auto saved=profile;EXPECT_FALSE(catalog.GlobalLaw1Profile(Family::T3,0,&profile));SameProfile(profile,saved);
  EXPECT_FALSE(catalog.GlobalLaw1Profile(Family::Qeph,1,&profile));SameProfile(profile,saved);
  EXPECT_FALSE(catalog.GlobalLaw1Profile(Family::None,0,&profile));SameProfile(profile,saved);
  EXPECT_FALSE(catalog.GlobalLaw1Profile(Family::Qeph,0,nullptr));
  tl::material::ShellElasticLaw1PointParameters point;point.elastic.a11=73;
  EXPECT_FALSE(catalog.ElasticParameters(Family::Qeph,0,&point));EXPECT_EQ(point.elastic.a11,73);
  fe::sections::PointParameters plastic;plastic.a11=42;
  EXPECT_FALSE(catalog.Parameters(Family::Qeph,0,&plastic));EXPECT_EQ(plastic.a11,42);
  ASSERT_TRUE(catalog.Parameters(Family::T3,0,&plastic));EXPECT_EQ(plastic.curve.yield_stress_pa[0],5400);
  const auto readback=fe::ShellBatchLayeredSection::GlobalLaw1();EXPECT_EQ(readback.law(),Law::GlobalLaw1Npt0);
  EXPECT_EQ(readback.elastic(),nullptr);EXPECT_EQ(readback.plastic(),nullptr);EXPECT_EQ(readback.one_point(),nullptr);
  // Explicit execution admission is mandatory; ordinary source/section APIs do
  // not infer global integration from an elastic MID or the zero point count.
  Catalog legacy;const auto before=Bytes(legacy);
  EXPECT_NE(legacy.Initialize(shells,source.Input()).status,Status::Success);EXPECT_EQ(Bytes(legacy),before);
  EXPECT_NE(legacy.InitializeSections(shells,source.Input()).status,Status::Success);EXPECT_EQ(Bytes(legacy),before);
  EXPECT_NE(legacy.InitializeSectionCatalog(shells,source.Input(),{}).status,Status::Success);EXPECT_EQ(Bytes(legacy),before);
  ASSERT_EQ(legacy.InitializeExecutionCatalog(shells,source.Input()).status,Status::Success);
}
TEST(GlobalLaw1ExecutionCatalog, CompletePolicyIdentityAndOwnedBackingSurviveBorrowedSourceDestruction) {
  std::unique_ptr<Catalog> retained,clone;
  {
    Source source;fe::ShellBatchBinding shells;ASSERT_EQ(shells.Initialize(source.collection()).status,fe::ShellBindingStatus::Success);
    Catalog first;ASSERT_EQ(first.InitializeExecutionCatalog(shells,source.Input()).status,Status::Success);
    retained=std::make_unique<Catalog>(first);clone=std::make_unique<Catalog>(*retained);
    EXPECT_TRUE(first.Matches(shells));EXPECT_TRUE(retained->SameScope(*clone));
    for(unsigned change=0;change<3;++change) {
      Source other;
      if(change==0)other.parents[1].execution={};
      if(change==1)other.parents[1].execution.global_law1.thickness=Thickness::Reference;
      if(change==2)other.parents[1].execution.global_law1.coefficient_working_length_m=std::nextafter(.001,1.);
      Catalog different;ASSERT_EQ(different.InitializeExecutionCatalog(shells,other.Input()).status,Status::Success);
      EXPECT_FALSE(first.SameScope(different));EXPECT_TRUE(different.Matches(shells));
    }
    source.parents[1].execution={};source.materials[0].young_pa=1;source.yt[0]=-1;
    EXPECT_TRUE(first.SameScope(*retained));
  }
  fe::ShellGlobalLaw1Profile policy;ASSERT_TRUE(retained->GlobalLaw1Profile(Family::Qeph,0,&policy));SameProfile(policy,Global().global_law1);
  fe::sections::PointParameters plastic;ASSERT_TRUE(retained->Parameters(Family::T3,0,&plastic));EXPECT_EQ(plastic.curve.yield_stress_pa[0],5400);
  EXPECT_TRUE(retained->SameScope(*clone));
}
TEST(GlobalLaw1ExecutionCatalog, OneSourcePartCannotSilentlyMixExecutionPolicies) {
  Source source(true);fe::ShellBatchBinding shells;ASSERT_EQ(shells.Initialize(source.collection()).status,fe::ShellBindingStatus::Success);
  for(unsigned fault=0;fault<3;++fault) {
    SCOPED_TRACE(fault);auto old=source.parents[1].execution;
    if(fault==0)source.parents[1].execution={};
    if(fault==1)source.parents[1].execution.global_law1.thickness=Thickness::Reference;
    if(fault==2)source.parents[1].execution.global_law1.coefficient_working_length_m=.01;
    Catalog target;const auto before=Bytes(target);const auto report=target.InitializeExecutionCatalog(shells,source.Input());
    EXPECT_EQ(report.status,Status::IdentityMismatch);EXPECT_EQ(report.entry,1u);EXPECT_EQ(Bytes(target),before);
    source.parents[1].execution=old;
    ASSERT_EQ(target.InitializeExecutionCatalog(shells,source.Input()).status,Status::Success);
    fe::ShellGlobalLaw1Profile q,t;ASSERT_TRUE(target.GlobalLaw1Profile(Family::Qeph,0,&q));ASSERT_TRUE(target.GlobalLaw1Profile(Family::T3,0,&t));SameProfile(q,t);
    for(auto family:{Family::Qeph,Family::T3}) {
      unsigned points=99;fe::ShellSectionCounts counts;
      ASSERT_TRUE(target.MaterialPointCount(family,0,&points));EXPECT_EQ(points,0u);
      ASSERT_TRUE(target.Counts(family,&counts));EXPECT_EQ(counts.law1,1u);EXPECT_EQ(counts.law1_global_npt0,1u);
    }
  }
}
TEST(GlobalLaw1ExecutionCatalog, UnsupportedFirstProfileCombinationsRejectAtomicallyAndRetry) {
  for(unsigned fault=0;fault<8;++fault) {
    SCOPED_TRACE(fault);Source source;
    if(fault==0)source.parents[1].execution.policy=static_cast<Policy>(99);
    if(fault==1)source.parents[1].execution.global_law1.coefficient_working_length_m=0;
    if(fault==2)source.parents[1].execution.global_law1.coefficient_working_length_m=std::numeric_limits<double>::quiet_NaN();
    if(fault==3)source.parents[1].execution.policy=Policy::FromSection; // Noncanonical leftover override.
    if(fault==4){source.materials[0].law=Law::LayeredLaw44Nip3;source.materials[0].curve_id=48;}
    if(fault==5)source.materials[0].law=Law::RigidSkin;
    if(fault==6){source.sections[0].through_thickness_points=1;source.sections[0].formulation=fe::ShellSectionFormulation::OneThicknessPoint;}
    if(fault==7)source.qeph.reference.placement=fe::ShellReferencePlacement::TopReferencePlane;
    fe::ShellBatchBinding shells;ASSERT_EQ(shells.Initialize(source.collection()).status,fe::ShellBindingStatus::Success);
    Catalog target;const auto before=Bytes(target);
    EXPECT_NE(target.InitializeExecutionCatalog(shells,source.Input()).status,Status::Success);EXPECT_EQ(Bytes(target),before);EXPECT_FALSE(target.prepared());
    Source good;fe::ShellBatchBinding good_shells;ASSERT_EQ(good_shells.Initialize(good.collection()).status,fe::ShellBindingStatus::Success);
    ASSERT_EQ(target.InitializeExecutionCatalog(good_shells,good.Input()).status,Status::Success);
  }
  // This is the first qualified source policy's boundary, not a claim that
  // native LAW1 integration is restricted to raw NIP3 in every formulation.
  qbat_catalog_test::Fixture qbat;fe::ShellBatchBinding shells;
  ASSERT_EQ(shells.InitializeFormulations(qbat.Geometry()).status,fe::ShellBindingStatus::Success);
  qbat.parents[0].execution=Global();Catalog rejected;const auto before=Bytes(rejected);
  EXPECT_NE(rejected.InitializeExecutionCatalog(shells,qbat.Input()).status,Status::Success);EXPECT_EQ(Bytes(rejected),before);
  qbat.parents[0].execution={};ASSERT_EQ(rejected.InitializeExecutionCatalog(shells,qbat.Input()).status,Status::Success);
}
TEST(GlobalLaw1ExecutionCatalog, ExactOwnedAndScratchCapsPreserveUnpreparedState) {
  Source source;fe::ShellBatchBinding shells;ASSERT_EQ(shells.Initialize(source.collection()).status,fe::ShellBindingStatus::Success);
  Catalog admitted;ASSERT_EQ(admitted.InitializeExecutionCatalog(shells,source.Input()).status,Status::Success);
  auto limits=fe::ShellPlasticityCatalogLimits{};limits.max_owned_bytes=admitted.host_bytes();limits.max_startup_scratch_bytes=admitted.startup_scratch_bytes();
  EXPECT_GT(limits.max_owned_bytes,0u);EXPECT_GT(limits.max_startup_scratch_bytes,0u);
  for(bool owned:{true,false}) {
    Catalog target;const auto before=Bytes(target);auto short_cap=limits;
    if(owned)--short_cap.max_owned_bytes;else --short_cap.max_startup_scratch_bytes;
    EXPECT_EQ(target.InitializeExecutionCatalog(shells,source.Input(),short_cap).status,Status::ResourceLimit);EXPECT_EQ(Bytes(target),before);
    ASSERT_EQ(target.InitializeExecutionCatalog(shells,source.Input(),limits).status,Status::Success);
    EXPECT_EQ(target.host_bytes(),limits.max_owned_bytes);EXPECT_EQ(target.startup_scratch_bytes(),limits.max_startup_scratch_bytes);
  }
}
TEST(GlobalLaw1ExecutionCatalog, FailureDeclarationsMustRetainTheCompleteExecutionPolicy) {
  Source source;fe::ShellBatchBinding shells;ASSERT_EQ(shells.Initialize(source.collection()).status,fe::ShellBindingStatus::Success);
  Catalog catalog;ASSERT_EQ(catalog.InitializeExecutionCatalog(shells,source.Input()).status,Status::Success);
  std::array<fe::ShellFailureParentInput,2> rows{};
  for(unsigned i=0;i<2;++i)rows[i].source=source.parents[i];
  rows[0].policy=fe::ShellFailurePolicy::ConstantAllPoints;rows[0].constant.failure_strain=2.5;
  for(unsigned fault=0;fault<3;++fault) {
    auto bad=rows;
    if(fault==0)bad[1].source.execution={};
    if(fault==1)bad[1].source.execution.global_law1.thickness=Thickness::Reference;
    if(fault==2)bad[1].source.execution.global_law1.coefficient_working_length_m=.01;
    fe::ShellBatchFailureBinding binding;const auto failed=binding.InitializeExecution(catalog,bad.data(),bad.size());
    EXPECT_EQ(failed.status,Status::IdentityMismatch);EXPECT_EQ(failed.entry,1u);EXPECT_FALSE(binding.prepared());EXPECT_EQ(binding.catalog(),nullptr);
    ASSERT_EQ(binding.InitializeExecution(catalog,rows.data(),rows.size()).status,Status::Success);
    EXPECT_TRUE(binding.Matches(catalog));EXPECT_TRUE(fe::SameShellParentExecution(binding.parent(1)->source.execution,Global()));
  }
}
} // namespace global_law1_execution_catalog_test
