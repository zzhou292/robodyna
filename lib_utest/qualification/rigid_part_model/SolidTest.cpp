// SPDX-License-Identifier: AGPL-3.0-or-later
#include "SolidFixture.h"

namespace rigid_part_solid_test {
TEST(RigidPartSolidCoefficients, EachRealSolidFamilyAdmitsZeroJMembersAndMissingProducerRetries) {
  for(unsigned family=0;family<3;++family) {
    Fixture f(family);
    r::NodalRigidPartAssemblyModel owner;
    fe::NodalCoefficientLedger missing;
    ASSERT_TRUE(missing.Initialize({&f.map}));
    EXPECT_EQ(owner.Initialize(f.topology,missing,{1000,.001}).status,r::PartAssemblyStatus::MissingCoefficient);
    EXPECT_FALSE(owner.prepared());
    ASSERT_TRUE(owner.Initialize(f.topology,f.ledger,{1000,.001}));
    EXPECT_EQ(owner.members().size(),family==2?4u:6u);
    double mass=0;
    for(const auto& p:f.points) mass+=p.mass;
    EXPECT_DOUBLE_EQ(owner.roots()[0].value.raw.ledger.part_mass,mass);
    EXPECT_EQ(owner.roots()[0].value.raw.ledger.native_member_inertia,0);
    EXPECT_GT(owner.roots()[0].value.principal.inertia.x,0);
    for(const auto& m:owner.members())
      EXPECT_EQ(f.ledger.nodes()[m.domain_node].coefficients.isotropic_inertia,0);
  }
}
} // namespace rigid_part_solid_test
