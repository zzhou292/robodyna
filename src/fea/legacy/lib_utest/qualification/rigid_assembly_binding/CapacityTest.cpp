// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/constraints/NodalRigidAssemblyBinding.h"
#include "../rigid_part_model/Fixture.h"

namespace rigid_binding_test {
TEST(RigidAssemblyBinding, TwentyPreparedRootsRetain796MembersAndExplicitLimits) {
  namespace fe=tl::fea;
  std::vector<std::size_t> counts(22,2);counts[0]=399;counts[1]=396;
  rigid_part_model_test::Fixture f(counts);
  auto ledger=f.Ledger();auto topology=f.Topology();
  fe::rigid::NodalRigidPartAssemblyModel parts;
  ASSERT_TRUE(parts.Initialize(*topology,ledger,f.units));
  fe::NodalRigidAssemblyBinding binding;
  fe::RigidBindingLimits limits;limits.max_members_per_group=795;
  EXPECT_EQ(binding.Initialize(parts,nullptr,limits).status,fe::RigidBindingStatus::ResourceLimit);
  ++limits.max_members_per_group;
  ASSERT_TRUE(binding.Initialize(parts,nullptr,limits));
  ASSERT_EQ(binding.groups().size(),20u);EXPECT_EQ(binding.groups()[0].member_count,796u);
  EXPECT_EQ(binding.members().size(),topology->member_count());
  for(std::size_t g=0;g<binding.groups().size();++g) {
    EXPECT_EQ(binding.groups()[g].mass_kg,parts.roots()[g].value.raw.mass);
    for(unsigned k=0;k<9;++k)
      EXPECT_EQ(binding.groups()[g].principal.axes.v[k],parts.roots()[g].value.principal.axes.v[k]);
  }
  RecordProperty("retained_bytes",std::to_string(binding.owned_payload_bytes()));
  RecordProperty("startup_bytes",std::to_string(binding.startup_payload_bytes()));
}
} // namespace rigid_binding_test
