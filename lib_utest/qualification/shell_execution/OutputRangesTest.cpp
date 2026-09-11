// SPDX-License-Identifier: MIT
#include "Fixture.h"
#include "lib_src/elements/ShellPhysicalOutputRanges.h"
#include <array>

namespace shell_execution_test {
struct OutputFixture : Fixture {
  fe::ShellBatchPlasticityBinding independent_catalog;
  fe::ShellExecutionBinding execution;
  fe::ShellBatchFailureBinding failure;
  fe::ShellPhysicalBinding physical;
  OutputFixture() {
    EXPECT_EQ(independent_catalog.InitializeExecutionCatalog(shells,source.Catalog()).status,Status::Success);
    EXPECT_EQ(execution.Initialize(independent_catalog,ledger,rigid).status,Status::Success);
    const auto rows=source.Failures();
    EXPECT_EQ(failure.InitializeExecution(catalog,rows.data(),rows.size()).status,Status::Success);
    EXPECT_TRUE(physical.InitializeExecution({&shells,&catalog,&failure,nullptr},ledger,execution));
  }
  bool Disjoint(const void* output,std::size_t size) const {
    return fe::shell_physical_owner::OutputDisjoint(physical,output,size);
  }
};
TEST(ShellExecutionRanges, RolesAndIndependentCatalogBackingAreExcluded) {
  OutputFixture f;
  std::array<std::byte,256> output{};
  ASSERT_TRUE(f.Disjoint(output.data(),output.size()));
  const auto& roles=*f.physical.execution();
  ASSERT_GE(roles.parents().size()*sizeof(fe::ShellExecutionParent),output.size());
  EXPECT_FALSE(f.Disjoint(roles.parents().data(),output.size()));
  EXPECT_FALSE(f.Disjoint(roles.parents().data()+roles.parents().size()-1,1));
  ASSERT_NE(roles.catalog()->parent(0),f.physical.catalog()->parent(0));
  EXPECT_FALSE(f.Disjoint(roles.catalog()->parent(roles.catalog()->parent_count()-1),1));
  EXPECT_FALSE(f.Disjoint(f.physical.catalog()->parent(0),1));
  const auto& inventory=roles.catalog()->inventory().words();
  EXPECT_FALSE(f.Disjoint(inventory.data()+inventory.size()-1,1));
  EXPECT_TRUE(f.Disjoint(output.data(),output.size()));
}
TEST(ShellExecutionRanges, CompletePartMergeAndPreparedCoefficientRangesAreExcluded) {
  OutputFixture f;
  const auto& rigid=*f.physical.execution()->rigid();
  const auto& parts=*rigid.parts();
  const auto& topology=*parts.topology();
  const void* source[]{&rigid,rigid.groups().data(),rigid.members().data(),
      &parts,parts.members().data(),parts.original_bodies().data(),parts.roots().data(),
      &topology,topology.parts(),topology.merges(),topology.roots(),
      topology.original_members(),topology.root_members(),topology.expected_members(),
      parts.coefficients(),rigid.coefficients(),f.physical.coefficients()->nodes().data(),
      f.physical.domain()->nodes().data(),f.physical.mapping()->mapping().data()};
  for (const auto* address:source) {
    ASSERT_NE(address,nullptr);
    EXPECT_FALSE(f.Disjoint(address,1));
  }
  EXPECT_FALSE(f.Disjoint(nullptr,1));
  fe::ShellPhysicalBinding empty;
  double output=5;
  EXPECT_FALSE(fe::shell_physical_owner::OutputDisjoint(empty,&output,sizeof(output)));
  EXPECT_EQ(output,5);
}
} // namespace shell_execution_test
