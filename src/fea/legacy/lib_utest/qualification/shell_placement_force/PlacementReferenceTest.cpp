#include "PlacementForceFixture.h"

namespace placement_force_test {
template<class F> void ReferenceIdentity() {
  Fixture<F> centered(Placement::Centered),top(Placement::TopReferencePlane),bottom(Placement::BottomReferencePlane);
  constexpr unsigned nodes=std::is_same_v<F,Q>?4:3;
  for(unsigned n=0;n<nodes;++n) {
    EXPECT_DOUBLE_EQ(top.reference.nodal_mass[n],centered.reference.nodal_mass[n]);
    EXPECT_DOUBLE_EQ(top.reference.isotropic_inertia[n],bottom.reference.isotropic_inertia[n]);
    EXPECT_DOUBLE_EQ(top.reference.physical_inertia[n],bottom.reference.physical_inertia[n]);
    EXPECT_DOUBLE_EQ(top.reference.added_inertia[n],centered.reference.added_inertia[n]);
    if constexpr(std::is_same_v<F,Q>) {
      EXPECT_GT(top.reference.isotropic_inertia[n],centered.reference.isotropic_inertia[n]);
      EXPECT_DOUBLE_EQ(top.reference.physical_inertia[n],top.reference.nodal_mass[n]*.00228*.00228*(1./12+.25));
    } else EXPECT_DOUBLE_EQ(top.reference.isotropic_inertia[n],centered.reference.isotropic_inertia[n]);
  }
  EXPECT_FALSE(top.accepted.shell.matches_reference(bottom.reference));
  EXPECT_TRUE(top.accepted.shell.matches_reference(top.reference));
  auto input=top.reference.input;
  input.placement=static_cast<Placement>(255);
  const auto saved=tab1_test::Bytes(top.reference);
  EXPECT_EQ(InitializeReference(input,top.reference),F::Status::kInvalidInput);
  EXPECT_EQ(tab1_test::Bytes(top.reference),saved);
}
TEST(PlacementReference,QephNativeOffsetMassPartitionAndExactIdentity) {ReferenceIdentity<Q>();}
TEST(PlacementReference,T3NativeUnchangedInertiaStillRequiresExactPlacementIdentity) {ReferenceIdentity<T>();}

TEST(PlacementReference,ExplicitPerParentInventoryWordsAndBoundedFailurePublication) {
  using namespace tl::fea;
  ShellBatchBinding a,b,c;
  const auto in=BindingInput(Placement::TopReferencePlane,Placement::BottomReferencePlane);
  ASSERT_EQ(a.Initialize(in).status,ShellBindingStatus::Success);
  ASSERT_EQ(b.Initialize(BindingInput(Placement::BottomReferencePlane,Placement::TopReferencePlane)).status,ShellBindingStatus::Success);
  ASSERT_EQ(c.Initialize(in).status,ShellBindingStatus::Success);
  EXPECT_EQ(a.inventory(),c.inventory());
  EXPECT_NE(a.inventory(),b.inventory());
  EXPECT_DOUBLE_EQ(a.totals().mass,b.totals().mass);
  EXPECT_DOUBLE_EQ(a.totals().isotropic_inertia,b.totals().isotropic_inertia);
  const auto words=a.inventory().words();
  ASSERT_EQ(words.size(),52u);
  EXPECT_EQ(words[0],6u);
  EXPECT_EQ(words[28],static_cast<unsigned>(Placement::TopReferencePlane));
  EXPECT_EQ(words[51],static_cast<unsigned>(Placement::BottomReferencePlane));
  ShellQephBindingInput q{in.qeph,in.qeph_nodes,21};
  ShellT3BindingInput t{in.t3,in.t3_nodes,22};
  ShellBatchCollectionInput collection{&q,&t,1,1,7};
  ShellBatchBinding limited;
  const auto limited_before=tab1_test::Bytes(limited);
  ShellHostBindingLimits limits;
  limits.max_owned_bytes=sizeof(ShellBatchBinding)-1;
  auto poisoned=collection;
  poisoned.qeph=reinterpret_cast<const ShellQephBindingInput*>(1);
  poisoned.t3=reinterpret_cast<const ShellT3BindingInput*>(1);
  EXPECT_EQ(limited.Initialize(poisoned,limits).status,ShellBindingStatus::ResourceLimit);
  EXPECT_EQ(tab1_test::Bytes(limited),limited_before);
  ASSERT_EQ(limited.Initialize(collection,{}).status,ShellBindingStatus::Success);
  ShellBatchBinding invalid;
  const auto before=tab1_test::Bytes(invalid);
  t.reference.placement=static_cast<Placement>(255);
  EXPECT_EQ(invalid.Initialize(collection).status,ShellBindingStatus::InvalidT3Reference);
  EXPECT_EQ(tab1_test::Bytes(invalid),before);
  t.reference.placement=in.t3.placement;
  ASSERT_EQ(invalid.Initialize(collection).status,ShellBindingStatus::Success);
  EXPECT_EQ(invalid.inventory().words().size(),56u);
  EXPECT_EQ(invalid.inventory().words()[0],7u);
}
} // namespace placement_force_test
