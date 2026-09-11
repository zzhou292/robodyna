// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"

namespace rigid_binding_test {
TEST(RigidAssemblyBinding, CopiesFinalizedPartAndPlainPropertiesWithoutRepeatingCorrection) {
  Fixture f;auto parts=f.Parts();auto plain=f.Plain();
  fe::NodalRigidAssemblyBinding binding;
  const auto report=binding.Initialize(parts,plain.get());
  ASSERT_TRUE(report)<<report.message;
  ASSERT_EQ(binding.groups().size(),2u);
  EXPECT_EQ(binding.plain_source_instance_id(),29u);
  EXPECT_TRUE(binding.coefficients()->Matches(f.solid.ledger));
  EXPECT_TRUE(binding.domain()->SharesStorage(f.solid.domain));
  const auto& a=binding.groups()[0];const auto& b=binding.groups()[1];
  EXPECT_EQ(a.source_kind,fe::RigidBindingSourceKind::Part);
  EXPECT_EQ(b.source_kind,fe::RigidBindingSourceKind::NodalGroup);
  EXPECT_EQ(a.source_id,b.source_id); // Equal numbers retain different namespaces.
  EXPECT_EQ(a.source_node_set_id,0u);EXPECT_EQ(b.source_node_set_id,501u);
  EXPECT_EQ(a.mass_kg,parts.roots()[0].value.raw.mass);
  EXPECT_EQ(b.mass_kg,plain->groups()[0].total_mass_kg);
  SameFrame(a.principal,parts.roots()[0].value.principal);
  SameFrame(b.principal,plain->groups()[0].principal);
  EXPECT_EQ(binding.parts()->roots()[0].value.regularization.primary_mass_kg,
      parts.roots()[0].value.regularization.primary_mass_kg);
  ASSERT_EQ(binding.members().size(),8u);
  for(std::size_t k=0;k<binding.members().size();++k) {
    const auto& member=binding.members()[k];
    EXPECT_EQ(binding.FindMember(member.domain_node),&member);
    const auto& c=f.solid.ledger.nodes()[member.domain_node].coefficients;
    EXPECT_EQ(member.mass_kg,c.mass);EXPECT_EQ(member.isotropic_inertia_kg_m2,c.isotropic_inertia);
    if(k<6)EXPECT_EQ(member.isotropic_inertia_kg_m2,0);
  }
  EXPECT_EQ(binding.FindMember(f.solid.domain.Find(777)),nullptr);
  EXPECT_EQ(binding.FindMember(SIZE_MAX),nullptr);
}
TEST(RigidAssemblyBinding, RejectsOmittedCensusLateCoefficientAndCoordinateMismatchThenRetries) {
  Fixture f;const auto parts=f.Parts();auto good=f.Plain();
  fe::NodalRigidAssemblyBinding omitted;
  EXPECT_EQ(omitted.Initialize(parts).status,S::IdentityMismatch);
  ASSERT_TRUE(omitted.Initialize(parts,good.get()));
  const auto changed=f.Parts(true);
  fe::NodalRigidAssemblyBinding wrong_count;
  EXPECT_EQ(wrong_count.Initialize(changed,good.get()).status,S::IdentityMismatch);
  f.plain_members[1].mass_kg=std::nextafter(f.plain_members[1].mass_kg,INFINITY);
  auto bad=f.Plain();fe::NodalRigidAssemblyBinding retry;
  const auto failure=retry.Initialize(parts,bad.get());
  EXPECT_EQ(failure.status,S::IdentityMismatch);EXPECT_EQ(failure.group,1u);EXPECT_EQ(failure.member,7u);
  EXPECT_FALSE(retry.prepared());ASSERT_TRUE(retry.Initialize(parts,good.get()));
  f.plain_members[1].mass_kg=good->members()[1].mass_kg;
  f.plain_members[1].position.z=-0.;
  auto signed_zero=f.Plain();fe::NodalRigidAssemblyBinding coordinate;
  EXPECT_EQ(coordinate.Initialize(parts,signed_zero.get()).status,S::IdentityMismatch);
}
TEST(RigidAssemblyBinding, RetainsBackingAfterInputsDieAndHonorsWholeStartupBudget) {
  fe::NodalRigidAssemblyBinding binding;
  {
    Fixture f;auto parts=f.Parts();auto plain=f.Plain();
    ASSERT_TRUE(binding.Initialize(parts,plain.get()));
    fe::RigidBindingLimits cap;cap.max_host_bytes=binding.startup_payload_bytes()-1;
    fe::NodalRigidAssemblyBinding retry;
    EXPECT_EQ(retry.Initialize(parts,plain.get(),cap).status,S::ResourceLimit);
    ++cap.max_host_bytes;ASSERT_TRUE(retry.Initialize(parts,plain.get(),cap));
    EXPECT_EQ(retry.Initialize(parts,plain.get(),cap).status,S::AlreadyInitialized);
  }
  EXPECT_EQ(binding.members().size(),8u);
  EXPECT_TRUE(binding.parts()->prepared());EXPECT_TRUE(binding.coefficients()->prepared());
  EXPECT_EQ(binding.FindMember(binding.members()[7].domain_node)->source_node_id,11u);
  auto copy=binding;EXPECT_EQ(copy.members().data(),binding.members().data());
  EXPECT_EQ(copy.owned_payload_bytes(),binding.owned_payload_bytes());
}
} // namespace rigid_binding_test
