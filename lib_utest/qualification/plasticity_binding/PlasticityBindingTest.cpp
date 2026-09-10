#include "PlasticityBindingFixture.h"
#include <limits>
#include <memory>
#include <utility>

namespace plasticity_binding_test {
TEST(ShellPlasticityBinding, CompleteNativeParentMappingOwnsDistinctMaterialsAndCurves) {
  Fixture f; fe::ShellBatchBinding native;
  ASSERT_EQ(native.Initialize(f.collection()).status,fe::ShellBindingStatus::Success);
  Binding catalog; ASSERT_EQ(catalog.Initialize(native,f.catalog()).status,Status::Success);
  ASSERT_TRUE(catalog.Matches(native)); EXPECT_EQ(catalog.curve_point_count(),6u);
  EXPECT_EQ(catalog.parent(0)->source_parent_id,702u); EXPECT_EQ(catalog.parent(1)->source_parent_id,701u);
  EXPECT_EQ(catalog.parent(2),nullptr);
  fe::sections::PointParameters qp,tp;
  ASSERT_TRUE(catalog.Parameters(fe::ShellBindingFamily::Qeph,0,&qp));
  ASSERT_TRUE(catalog.Parameters(fe::ShellBindingFamily::T3,0,&tp));
  EXPECT_EQ(qp.young_pa,2e6); EXPECT_EQ(tp.young_pa,3e6);
  EXPECT_EQ(qp.curve.yield_stress_pa[0],2700); EXPECT_EQ(tp.curve.yield_stress_pa[0],5400);
  EXPECT_FALSE(qp.rate.enabled); EXPECT_TRUE(tp.rate.enabled);
  f.x[1]=-1; f.yq[0]=-1; f.yt[0]=-1; f.materials[1].rate={}; f.parents[0].material_id=37;
  EXPECT_EQ(qp.curve.plastic_strain[1],.1); EXPECT_EQ(qp.curve.yield_stress_pa[0],2700);
  EXPECT_EQ(tp.curve.yield_stress_pa[0],5400);
  Binding copied(catalog); EXPECT_TRUE(copied.SameScope(catalog));
  fe::sections::PointParameters copy;
  ASSERT_TRUE(copied.Parameters(fe::ShellBindingFamily::T3,0,&copy));
  EXPECT_NE(copy.curve.plastic_strain,tp.curve.plastic_strain);
  EXPECT_EQ(copy.curve.yield_stress_pa[1],6800);
  const auto* copied_pool=copy.curve.plastic_strain;
  Binding moved(std::move(copied)); ASSERT_TRUE(moved.Parameters(fe::ShellBindingFamily::T3,0,&copy));
  EXPECT_NE(copy.curve.plastic_strain,copied_pool);
  EXPECT_NE(copy.curve.plastic_strain,tp.curve.plastic_strain); EXPECT_EQ(copy.curve.yield_stress_pa[1],6800);
  const auto held=Bytes(copy);
  EXPECT_FALSE(moved.Parameters(fe::ShellBindingFamily::T3,1,&copy)); EXPECT_EQ(Bytes(copy),held);
  EXPECT_FALSE(moved.Parameters(fe::ShellBindingFamily::None,0,&copy)); EXPECT_EQ(Bytes(copy),held);
  EXPECT_FALSE(moved.Parameters(fe::ShellBindingFamily::Qeph,0,nullptr));
  const auto prepared=Bytes(catalog);
  EXPECT_EQ(catalog.Initialize(native,f.catalog()).status,Status::AlreadyInitialized); EXPECT_EQ(Bytes(catalog),prepared);
}
TEST(ShellPlasticityBinding, ParentIndexSelectsDistinctMaterialsWithinTheSameFamily) {
  Fixture f; std::array<fe::ShellQephBindingInput,2> qeph{{f.qeph,f.qeph}};
  qeph[1].source_parent_id=703;
  auto& second=qeph[1].reference;
  second.young_modulus=3e6; second.poisson_ratio=.25; second.density=768; second.thickness=1./64;
  fe::ShellBatchBinding native;
  ASSERT_EQ(native.Initialize({qeph.data(),&f.t3,2,1,5}).status,fe::ShellBindingStatus::Success);
  const fe::ShellPlasticityParentInput parents[]{f.parents[0],
      {fe::ShellBindingFamily::Qeph,1,703,83,38,58},f.parents[1]};
  auto input=f.catalog(); input.parents=parents; input.parent_count=3;
  Binding catalog; ASSERT_EQ(catalog.Initialize(native,input).status,Status::Success);
  fe::sections::PointParameters first,later;
  ASSERT_TRUE(catalog.Parameters(fe::ShellBindingFamily::Qeph,0,&first));
  ASSERT_TRUE(catalog.Parameters(fe::ShellBindingFamily::Qeph,1,&later));
  EXPECT_EQ(first.young_pa,2e6); EXPECT_EQ(later.young_pa,3e6);
  EXPECT_EQ(first.curve.yield_stress_pa[0],2700); EXPECT_EQ(later.curve.yield_stress_pa[0],5400);
  EXPECT_EQ(catalog.parent(1)->source_parent_id,703u);
}

template<class Change> void Rejected(Change change,Status expected) {
  Fixture f; fe::ShellBatchBinding native;
  ASSERT_EQ(native.Initialize(f.collection()).status,fe::ShellBindingStatus::Success);
  auto input=f.catalog(); change(f,input); Binding catalog; const auto old=Bytes(catalog);
  EXPECT_EQ(catalog.Initialize(native,input).status,expected); EXPECT_EQ(Bytes(catalog),old);
  Fixture clean; ASSERT_EQ(catalog.Initialize(native,clean.catalog()).status,Status::Success);
}
TEST(ShellPlasticityBinding, DuplicateMissingAndUnreferencedDeclarationsRejectAtomically) {
  Rejected([](auto& f,auto&){f.curves[1].curve_id=47;},Status::InvalidCurve);
  Rejected([](auto& f,auto&){f.materials[1].material_id=37;},Status::InvalidMaterial);
  Rejected([](auto& f,auto&){f.sections[1].section_id=57;},Status::InvalidSection);
  Rejected([](auto& f,auto&){f.parents[1]=f.parents[0];},Status::InvalidParent);
  Rejected([](auto&,auto& in){in.parent_count=1;},Status::InvalidParent);
  Rejected([](auto& f,auto&){f.materials[1].curve_id=999;},Status::InvalidMaterial);
  Rejected([](auto& f,auto&){f.parents[0].material_id=999;},Status::InvalidParent);
  Rejected([](auto& f,auto&){f.parents[0].section_id=999;},Status::InvalidParent);
  Rejected([](auto& f,auto&){f.materials[1].curve_id=47;},Status::UnreferencedDeclaration);
  Rejected([](auto& f,auto&){f.parents[0].source_part_id=81;},Status::IdentityMismatch);
  Rejected([](auto& f,auto&){f.parents[0].source_parent_id=701;},Status::IdentityMismatch);
  Rejected([](auto& f,auto&){f.parents[0].source_part_id=0;},Status::InvalidParent);
  Rejected([](auto& f,auto&){f.parents[0].family_index=1;},Status::InvalidParent);
}
TEST(ShellPlasticityBinding, NativeReferenceBitsAndLateCurveFailureCannotBeSubstituted) {
  Rejected([](auto& f,auto&){f.materials[1].young_pa+=1;},Status::IdentityMismatch);
  Rejected([](auto& f,auto&){f.materials[1].poisson_ratio+=.01;},Status::IdentityMismatch);
  Rejected([](auto& f,auto&){f.materials[1].density_kg_m3+=1;},Status::IdentityMismatch);
  Rejected([](auto& f,auto&){f.sections[1].thickness_m*=2;},Status::IdentityMismatch);
  Rejected([](auto& f,auto&){f.sections[1].through_thickness_points=5;},Status::InvalidSection);
  Rejected([](auto& f,auto&){f.materials[1].rate.cutoff_hz=0;},Status::InvalidMaterial);
  Rejected([](auto& f,auto&){f.yt[2]=std::numeric_limits<double>::quiet_NaN();},Status::InvalidCurve);
}
TEST(ShellPlasticityBinding, UnreferencedMaterialAndSectionRejectWithoutPublication) {
  Fixture f; fe::ShellBatchBinding native;
  ASSERT_EQ(native.Initialize(f.collection()).status,fe::ShellBindingStatus::Success);
  const fe::ShellPlasticityMaterialInput materials[]{f.materials[0],f.materials[1],{39,47,2e6,.3,1024,{}}};
  const fe::ShellPlasticitySectionInput sections[]{f.sections[0],f.sections[1],{59,1./32,3}};
  for(bool extra_material:{false,true}) {
    auto input=f.catalog();
    if(extra_material) { input.materials=materials; input.material_count=3; }
    else { input.sections=sections; input.section_count=3; }
    Binding catalog; const auto old=Bytes(catalog);
    EXPECT_EQ(catalog.Initialize(native,input).status,Status::UnreferencedDeclaration);
    EXPECT_EQ(Bytes(catalog),old);
    EXPECT_EQ(catalog.Initialize(native,f.catalog()).status,Status::Success);
  }
}
TEST(ShellPlasticityBinding, CompleteScopeIncludesOtherFamilyMaterialAndParentOrder) {
  Fixture first,second; fe::ShellBatchBinding native,changed;
  ASSERT_EQ(native.Initialize(first.collection()).status,fe::ShellBindingStatus::Success);
  Binding a; ASSERT_EQ(a.Initialize(native,first.catalog()).status,Status::Success);
  second.yq[1]+=1; Binding b; ASSERT_EQ(b.Initialize(native,second.catalog()).status,Status::Success);
  EXPECT_FALSE(a.SameScope(b)); // T3 unchanged; full QEPH declaration still belongs to its scope.
  Fixture order; std::swap(order.parents[0],order.parents[1]); Binding c;
  ASSERT_EQ(c.Initialize(native,order.catalog()).status,Status::Success); EXPECT_FALSE(a.SameScope(c));
  Fixture shifted; shifted.Translate(.125);
  ASSERT_EQ(changed.Initialize(shifted.collection()).status,fe::ShellBindingStatus::Success);
  EXPECT_FALSE(a.Matches(changed)); Binding d;
  ASSERT_EQ(d.Initialize(changed,shifted.catalog()).status,Status::Success); EXPECT_FALSE(a.SameScope(d));
  Binding empty; EXPECT_FALSE(a.SameScope(empty)); EXPECT_FALSE(empty.Matches(native));
}
TEST(ShellPlasticityBinding, PoolCapUsesTotalPointsAndChecksOverflowBeforeCopies) {
  Rejected([](auto&,auto& in){in.material_count=std::numeric_limits<std::size_t>::max();},Status::ResourceLimit);
  Rejected([](auto& f,auto&){f.curves[1].curve.count=UINT32_MAX;},Status::ResourceLimit);
  Rejected([](auto& f,auto&){f.curves[0].curve.count=1024;f.curves[1].curve.count=2;},Status::ResourceLimit);
  Fixture f; fe::ShellBatchBinding native; ASSERT_EQ(native.Initialize(f.collection()).status,fe::ShellBindingStatus::Success);
  std::array<double,512> x{},y{};
  for(unsigned i=0;i<512;++i) {x[i]=double(i)/511.; y[i]=2700.+i;}
  for(auto& c:f.curves) c.curve={x.data(),y.data(),512};
  Binding boundary; ASSERT_EQ(boundary.Initialize(native,f.catalog()).status,Status::Success);
  EXPECT_EQ(boundary.curve_point_count(),1024u);
}
} // namespace plasticity_binding_test
