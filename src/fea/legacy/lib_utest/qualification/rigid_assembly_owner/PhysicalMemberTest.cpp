// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include <limits>

namespace rigid_assembly_owner_test {
TEST(RigidPhysicalMembers,ZeroInertiaPointAndSolidLikeMembersRetainRealTensorAndNativeRegularization) {
  std::array<fe::NodalRigidGroupMember,4> members{{
      {10,0,{0,0,0},0,0,0,0}, {11,1,{.2,0,0},2,0,0,0},
      {12,2,{0,.3,0},3,0,0,0}, {13,3,{0,0,.4},4,0,0,0}}};
  const fe::NodalRigidGroupInput group{101,201,members.data(),members.size()};
  const fe::NodalRigidGroupModelInput input{1,4,&group,1,{1000,.001}};
  fe::NodalRigidGroupModel strict,physical;
  EXPECT_EQ(strict.Initialize(input).status,fe::NodalRigidGroupStatus::InvalidMass);
  ASSERT_TRUE(physical.InitializePhysical(input));
  EXPECT_TRUE(physical.physical_coefficients());
  const auto& g=physical.groups()[0];
  EXPECT_EQ(g.structural_mass_kg,9);
  EXPECT_EQ(g.native_total_inertia_sum,0);
  EXPECT_EQ(g.physical_inertia_sum,0);
  EXPECT_EQ(g.regularization.primary_mass_kg,1e-20*1000);
  EXPECT_EQ(g.regularization.primary_isotropic_inertia_kg_m2,(1e-20*1000)*.001*.001);
  long double tensor[9]{};
  auto add=[&](tl::math::Vec3 p,long double mass,long double inertia) {
    const long double r[]{(long double)p.x-g.center.x,(long double)p.y-g.center.y,(long double)p.z-g.center.z};
    for(unsigned a=0;a<3;++a) for(unsigned b=0;b<3;++b)
      tensor[3*a+b]+=(a==b?inertia+mass*(r[0]*r[0]+r[1]*r[1]+r[2]*r[2]):0)-mass*r[a]*r[b];
  };
  add(g.generated_primary_position,g.regularization.primary_mass_kg,g.regularization.primary_isotropic_inertia_kg_m2);
  for(const auto& m:members) add(m.position,m.mass_kg,m.total_inertia_kg_m2);
  for(unsigned k=0;k<9;++k) EXPECT_NEAR(g.raw_tensor.v[k],double(tensor[k]),2e-16);
}
TEST(RigidPhysicalMembers,AllZeroOrNegativeSourceCannotBecomeARegularizationOnlyBody) {
  std::array<fe::NodalRigidGroupMember,2> members{{{10,0,{0,0,0},0,0,0,0},{11,1,{1,0,0},0,0,0,0}}};
  const fe::NodalRigidGroupInput group{101,201,members.data(),members.size()};
  const fe::NodalRigidGroupModelInput input{1,2,&group,1,{1000,.001}};
  fe::NodalRigidGroupModel model;
  EXPECT_EQ(model.InitializePhysical(input).status,fe::NodalRigidGroupStatus::InvalidMass);
  EXPECT_FALSE(model.prepared());
  members[1].mass_kg=-1;
  EXPECT_EQ(model.InitializePhysical(input).status,fe::NodalRigidGroupStatus::InvalidMass);
  members[1].mass_kg=2;
  members[1].total_inertia_kg_m2=.1;
  EXPECT_EQ(model.InitializePhysical(input).status,fe::NodalRigidGroupStatus::InvalidMass);
  members[1].physical_inertia_kg_m2=.1;
  ASSERT_TRUE(model.InitializePhysical(input));
  EXPECT_EQ(model.members()[0].mass_kg,0);
  EXPECT_EQ(model.members()[0].total_inertia_kg_m2,0);
}
TEST(RigidPhysicalMembers,ActualPhysicalLedgerBindingRetainsPlainSourceKindAndZeroInertiaRole) {
  Fixture fixture(false,true);
  ASSERT_TRUE(fixture.plain.physical_coefficients());
  ASSERT_TRUE(fixture.binding.prepared());
  const auto& group=fixture.binding.groups()[1];
  EXPECT_EQ(group.source_kind,fe::RigidBindingSourceKind::NodalGroup);
  EXPECT_TRUE(group.dependent_coefficients);
  const auto node=fixture.domain.Find(778);
  ASSERT_NE(fixture.binding.FindMember(node),nullptr);
  EXPECT_EQ(fixture.binding.FindMember(node)->mass_kg,2);
  EXPECT_EQ(fixture.binding.FindMember(node)->isotropic_inertia_kg_m2,0);
  nd::RigidStorageLayout layout;
  EXPECT_EQ(nd::ForecastRigidStorage(fixture.binding,fixture.Config(),layout).status,Code::Ok);
}
} // namespace rigid_assembly_owner_test
