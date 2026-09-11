#include "Fixture.h"
#include "lib_src/elements/ShellBatchPlasticityBinding.h"

namespace qbat_binding_test {
TEST(QbatBinding, DistinctThreeLayersShareNodesAndContributeExactlyOnce) {
  Fixture f;
  Binding binding;
  ASSERT_EQ(binding.InitializeFormulations(f.Input()).status,Status::Success);
  ASSERT_EQ(binding.qeph_count(),2u);
  ASSERT_EQ(binding.t3_count(),1u);
  ASSERT_EQ(binding.qbat_count(),1u);
  ASSERT_EQ(binding.node_count(),5u);
  EXPECT_EQ(binding.qbat_source_id(0),103u);
  EXPECT_EQ(binding.qbat_nodes(0),f.b.nodes);
  Reduction(binding);
  // Independent physical layer mass on a rectangular Q4. No native frame or
  // reference helper participates in this once-only physical-mass expectation.
  const long double area=.04L*.02L;
  const long double expected=2*2500.L*.002L*area/4+1000.L*.0005L*area/4;
  EXPECT_NEAR(binding.nodes()[0].native.mass,static_cast<double>(expected),1e-17);
  EXPECT_NEAR(binding.qbat_totals().mass,static_cast<double>(1000.L*.0005L*area),1e-18);
  EXPECT_FALSE(binding.qeph_reference().prepared); // Not the historical pair.
  EXPECT_FALSE(binding.qbat_reference(1).prepared());
  EXPECT_EQ(binding.qbat_source_id(1),0u);
}

TEST(QbatBinding, CompleteFormulationOptionBitsAndPlacementAreImmutableIdentity) {
  Fixture f;
  Binding original;
  ASSERT_EQ(original.InitializeFormulations(f.Input()).status,Status::Success);
  const auto words=original.inventory().words();
  ASSERT_EQ(words.size(),5u+2*28+23+46);
  EXPECT_EQ(words[0],5u);
  EXPECT_EQ(words[1],5u);
  EXPECT_EQ(words[2],2u);
  EXPECT_EQ(words[3],1u);
  EXPECT_EQ(words[4],1u);
  const std::size_t offset=5+2*28+23;
  EXPECT_EQ(words[offset],11u);
  EXPECT_EQ(words[offset+1],4u);
  EXPECT_EQ(words[offset+2],103u);
  const std::uint64_t options[]{11,0,2,2,2,1,1,0,0,44,1,1,1};
  for(unsigned n=0;n<13;++n) EXPECT_EQ(words[offset+28+n],options[n]);
  for(unsigned n=0;n<4;++n) EXPECT_EQ(words[offset+41+n],Bits(0.));
  EXPECT_EQ(words[offset+45],Bits(f.b.reference.initial_a11_pa));
  for(unsigned change=0;change<7;++change) {
    auto altered=f;
    if(change==0) altered.b.source_parent_id++;
    if(change==1) altered.b.reference.initial_a11_pa=std::nextafter(altered.b.reference.initial_a11_pa,INFINITY);
    if(change==2) altered.b.reference.options.membrane_viscosity=.03;
    if(change==3) altered.b.reference.options.numerical_viscosity=.02;
    if(change==4) altered.b.reference.options.offset_ratio=-0.;
    if(change==5) altered.b.reference.options.inertia_denominator_override=-0.;
    if(change==6) altered.q[0].reference.placement=fe::ShellReferencePlacement::Centered;
    Binding other;
    ASSERT_EQ(other.InitializeFormulations(altered.Input()).status,Status::Success)<<change;
    EXPECT_NE(original.inventory(),other.inventory())<<change;
  }
}

TEST(QbatBinding, CopiedHandlesOwnCompleteQbatReferenceAfterBorrowedInputsExpire) {
  const auto create=[] {
    Fixture f;
    Binding b;
    EXPECT_EQ(b.InitializeFormulations(f.Input()).status,Status::Success);
    f.b.reference.quadrilateral.position[3].x=900.;
    f.b.source_parent_id=999;
    return b;
  };
  const auto first=create();
  Binding copy(first),moved(std::move(copy));
  EXPECT_EQ(&first.qbat_reference(0),&moved.qbat_reference(0));
  EXPECT_EQ(first.inventory(),copy.inventory());
  EXPECT_EQ(moved.qbat_source_id(0),103u);
  EXPECT_EQ(moved.qbat_reference(0).input().quadrilateral.position[3].x,0.);
  Reduction(moved);
  const auto before=Bytes(moved);
  Fixture f;
  EXPECT_EQ(moved.InitializeFormulations(f.Input()).status,Status::AlreadyInitialized);
  EXPECT_EQ(before,Bytes(moved));
}

TEST(QbatBinding, ExistingCatalogCannotDropQbatAndPretendCompleteScope) {
  Fixture f;
  Binding binding;
  ASSERT_EQ(binding.InitializeFormulations(f.Input()).status,Status::Success);
  fe::ShellBatchPlasticityBinding catalog;
  const auto before=Bytes(catalog);
  const fe::ShellBatchPlasticityBindingInput empty;
  const auto check=[](const fe::ShellPlasticityBindingReport& report) {
    EXPECT_EQ(report.status,fe::ShellPlasticityBindingStatus::InvalidInput);
    EXPECT_STREQ(report.message,"QBAT requires an explicit complete formulation catalog");
  };
  check(catalog.Initialize(binding,empty));
  check(catalog.InitializeCatalog(binding,empty,{}));
  check(catalog.InitializeSections(binding,empty));
  check(catalog.InitializeSectionCatalog(binding,empty,{}));
  EXPECT_EQ(before,Bytes(catalog));
}
} // namespace qbat_binding_test
