// SPDX-License-Identifier: MIT
#include "Probes.h"
#include "../shell_execution/Fixture.h"
namespace shell_output_range_test {
TEST(ShellOutputExecution, ExpandedCatalogAndEmptyRigidMatchFrozenAndPreserveGaps) {
  for(std::size_t copies:{1u,22u,65u}) {
    Collection f(copies);ASSERT_TRUE(f.ready());Execution e(f.binding,f.catalog);ASSERT_TRUE(e.value.prepared());
    CatalogProbes(*e.value.catalog(),[&](Probe p){SameExecution(e.value,p);});
    CatalogProbes(*f.failure.catalog(),[&](Probe p){SameExecution(e.value,p);});
    for(auto p:Around(e.value.parents().data(),e.value.parents().size()*sizeof(fe::ShellExecutionParent)))SameExecution(e.value,p);
    for(auto p:Around(e.value.rigid(),sizeof(fe::NodalRigidAssemblyBinding)))SameExecution(e.value,p);
    for(auto p:Around(e.value.coefficients()->nodes().data(),e.value.coefficients()->nodes().size()*sizeof(fe::NodalCoefficientNode)))SameExecution(e.value,p);
    const auto begin=Address(e.value.catalog()->parent(0))+sizeof(fe::ShellPlasticityParentInput);
    const auto end=Address(e.value.catalog()->parent(1));ASSERT_GT(end,begin);
    EXPECT_EQ(fe::shell_execution_detail::frozen341::OutputDisjoint(e.value,Pointer(begin),end-begin),copies>1);
    SameExecution(e.value,{Pointer(begin),static_cast<std::size_t>(end-begin)});
    fe::ShellExecutionBinding copy(e.value),moved(std::move(copy));
    ASSERT_TRUE(copy.prepared());ASSERT_EQ(copy.catalog(),e.value.catalog());
    for(auto p:Extreme()){SameExecution(copy,p);SameExecution(moved,p);}
  }
}
TEST(ShellOutputExecution, PreparedPartTopologyAndUnpreparedOrMissingHandlesKeepOldSemantics) {
  shell_execution_test::Fixture f;fe::ShellExecutionBinding execution;
  ASSERT_EQ(execution.Initialize(f.catalog,f.ledger,f.rigid).status,fe::ShellPlasticityBindingStatus::Success);
  CatalogProbes(*execution.catalog(),[&](Probe p){SameExecution(execution,p);});
  const auto& rigid=*execution.rigid();const auto& parts=*rigid.parts();const auto& topology=*parts.topology();
  const void* protected_ranges[]{&rigid,rigid.groups().data(),rigid.members().data(),&parts,
      parts.members().data(),parts.original_bodies().data(),parts.roots().data(),&topology,
      topology.parts(),topology.merges(),topology.roots(),topology.original_members(),
      topology.root_members(),topology.expected_members(),parts.coefficients(),rigid.coefficients()};
  for(const auto* address:protected_ranges) {
    ASSERT_NE(address,nullptr);SameExecution(execution,{address,1});
    EXPECT_FALSE(fe::shell_execution_detail::OutputDisjoint(execution,address,1));
  }
  fe::ShellExecutionBinding empty;fe::ShellBatchBinding shells;
  Catalog catalog;fe::ShellBatchFailureBinding failure;fe::NodalMassBinding mass;
  std::array<unsigned char,64> output{};output.fill(0xc3);const auto before=output;
  Collection prepared;ASSERT_TRUE(prepared.ready());auto scope=prepared.Scope();scope.mass=&mass;
  auto cases=Extreme();cases.push_back({output.data(),output.size()});
  for(auto p:cases) {
    SameExecution(empty,p);SameBinding(shells,p);
    SameScope({},p);SameScope({&shells,&catalog,nullptr,nullptr},p);
    SameScope({&shells,&catalog,&failure,nullptr},p);
    SameScope(scope,p);
  }
  EXPECT_EQ(output,before);
}
}
