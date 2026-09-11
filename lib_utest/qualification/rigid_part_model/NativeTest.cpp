// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "../nodal_rigid_group/assembly/AssemblyNativeOracle.h"

extern "C" void nodal_rigid_native_ispher2(const double*,double*,int*);
namespace rigid_part_model_test {
TEST(RigidPartModelNative, IndependentRawBodiesMergesAndPrimaryExtraReset) {
  for(const double primary_scale:{1000.,1e20}) {
    Fixture f;f.units.mass_to_kg=primary_scale;
    const auto ledger=f.Ledger();const auto topology=f.Topology();
    Model model;ASSERT_TRUE(model.Initialize(*topology,ledger,f.units));
    std::vector<rigid_assembly_test::NativeValues> original;
    for(std::size_t p=0;p<f.part.size();++p) {
      const Packet packet(f,ledger,p);
      original.push_back(rigid_assembly_test::NativeRaw(packet.Input()));
      const auto actual=RawValues(model.original_bodies()[p].raw);
      for(unsigned k=0;k<actual.size();++k) EXPECT_DOUBLE_EQ(actual[k],original.back()[k])<<p<<","<<k;
    }
    for(std::size_t i=0;i<model.roots().size();++i) {
      const auto& root=topology->roots()[i];
      const auto native=rigid_assembly_test::NativeMerge(original[root.part_index],original[root.child_part_index]);
      const auto actual=RawValues(model.roots()[i].value.raw);
      for(unsigned k=0;k<actual.size();++k) EXPECT_DOUBLE_EQ(actual[k],native[k])<<i<<","<<k;
      EXPECT_EQ(model.roots()[i].value.raw.ledger.primaries,2u);
    }
  }
}
TEST(RigidPartModelNative, FinalCorrectionMatchesQualifiedNativeThresholdAfterMerge) {
  Fixture f({2,2});
  f.extra[0].clear(); // Two raw line-like bodies; the source-directed merge stays RAW.
  const auto ledger=f.Ledger();const auto topology=f.Topology();
  Model model;ASSERT_TRUE(model.Initialize(*topology,ledger,f.units));
  bool original_would_change=false;
  for(const auto& row:model.original_bodies()) {
    r::AssemblyFinalBody premature;
    ASSERT_TRUE(r::FinalizeAssemblyRawBody(row.raw,premature));
    original_would_change|=premature.regularization.principal_inertia_changed;
  }
  EXPECT_TRUE(original_would_change); // Distinguishes correction before versus after merging.
  for(const auto& row:model.roots()) {
    const auto& value=row.value;
    const double input[]{value.raw_principal_inertia.x,value.raw_principal_inertia.y,value.raw_principal_inertia.z};
    double output[3]{};int changed=-1;
    // Eigen axes are not compared to VALPR bits. This independently qualifies
    // the native nonlinear correction applied to those raw principal values.
    nodal_rigid_native_ispher2(input,output,&changed);
    EXPECT_EQ(Bits(value.principal.inertia.x),Bits(output[0]));
    EXPECT_EQ(Bits(value.principal.inertia.y),Bits(output[1]));
    EXPECT_EQ(Bits(value.principal.inertia.z),Bits(output[2]));
    EXPECT_EQ(value.regularization.principal_inertia_changed,changed!=0);
  }
}
} // namespace rigid_part_model_test
