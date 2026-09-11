// SPDX-License-Identifier: AGPL-3.0-or-later
#include "SolidFixture.h"
#include "Fixture.h"
#include "../nodal_rigid_group/assembly/AssemblyNativeOracle.h"

namespace rigid_part_solid_test {
TEST(RigidPartSolidNative, AllThreeSolidOnlyBodiesMatchIndependentNativeRawAssembly) {
  for(unsigned family=0;family<3;++family) {
    Fixture f(family);
    r::NodalRigidPartAssemblyModel model;
    ASSERT_TRUE(model.Initialize(f.topology,f.ledger,{1000,.001}));
    const auto expected=rigid_assembly_test::NativeRaw(f.RawInput());
    const auto actual=rigid_part_model_test::RawValues(model.original_bodies()[0].raw);
    for(unsigned k=0;k<actual.size();++k) EXPECT_DOUBLE_EQ(actual[k],expected[k])<<family<<","<<k;
  }
}
} // namespace rigid_part_solid_test
