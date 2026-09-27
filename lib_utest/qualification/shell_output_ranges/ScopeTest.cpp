// SPDX-License-Identifier: MIT
#include "Probes.h"
namespace shell_output_range_test {
TEST(ShellOutputScope, CompleteInterleavedCatalogFailureAndCurveChecksMatchFrozen) {
  for(std::size_t copies:{1u,22u,65u}) {
    Collection f(copies);ASSERT_TRUE(f.ready());const auto scope=f.Scope();
    CatalogProbes(*f.failure.catalog(),[&](Probe p){SameScope(scope,p);});
    CatalogProbes(f.catalog,[&](Probe p){SameScope(scope,p);});
    BindingProbes(f.binding,[&](Probe p){SameScope(scope,p);});
    for(auto p:Around(&f.failure,sizeof(f.failure)))SameScope(scope,p);
    for(std::size_t i=0;i<f.failure.parent_count();++i)
      for(auto p:Around(f.failure.parent(i),sizeof(fe::ShellFailureParentInput)))SameScope(scope,p);
    for(auto p:Extreme())SameScope(scope,p);
    // A query can cross two independent heap allocations. Arithmetic uses
    // address integers only; neither output nor the in-between gap is read.
    const auto a=Address(f.failure.catalog()->parent(0)),b=Address(f.failure.parent(0));
    const auto low=std::min(a,b),high=std::max(a,b);
    SameScope(scope,{Pointer(low),static_cast<std::size_t>(high-low+1)});
    std::array<unsigned char,64> external{};external.fill(0xa7);const auto before=external;
    EXPECT_TRUE(fe::shell_formulation_detail::OutputDisjoint(scope,external.data(),external.size()));
    SameScope(scope,{external.data(),external.size()});EXPECT_EQ(external,before);
  }
}
TEST(ShellOutputScope, CatalogStrideAndPrivateIndexGapsKeepOldAcceptance) {
  for(std::size_t copies:{1u,22u,65u}) {
    Collection f(copies);ASSERT_TRUE(f.ready());const auto scope=f.Scope();const auto& catalog=*f.failure.catalog();
    const auto begin=Address(catalog.parent(0))+sizeof(fe::ShellPlasticityParentInput);
    const auto end=Address(catalog.parent(1));ASSERT_GT(end,begin);
    const Probe gap{Pointer(begin),static_cast<std::size_t>(end-begin)};
    EXPECT_EQ(fe::shell_formulation_detail::frozen341::OutputDisjoint(scope,gap.pointer,gap.bytes),copies>1);
    SameScope(scope,gap);
    SameScope(scope,{Pointer(begin),0});SameScope(scope,{Pointer(begin-1),2});SameScope(scope,{Pointer(end-1),2});
  }
}
TEST(ShellOutputScope, IndependentFailureCatalogAndCopyMoveScopesRetainTheirOwnBacking) {
  for(std::size_t copies:{1u,65u}) {
    Collection f(copies);ASSERT_TRUE(f.ready());
    Catalog independent;
    ASSERT_EQ(independent.InitializeExecutionCatalog(f.binding,f.Input()).status,fe::ShellPlasticityBindingStatus::Success);
    fe::ShellBatchFailureBinding other;
    ASSERT_EQ(other.InitializeExecution(independent,f.failures.data(),f.failures.size()).status,fe::ShellPlasticityBindingStatus::Success);
    Catalog copied(f.catalog),moved(std::move(copied));ASSERT_TRUE(copied.prepared());
    fe::ShellBatchFailureBinding failure_copy(other),failure_moved(std::move(failure_copy));
    const fe::ShellFormulationScope scope{&f.binding,&moved,&failure_moved,nullptr};
    ASSERT_NE(failure_moved.catalog()->parent(0),f.failure.catalog()->parent(0));
    EXPECT_TRUE(fe::shell_formulation_detail::frozen341::OutputDisjoint(scope,f.catalog.parent(0),1));
    EXPECT_FALSE(fe::shell_formulation_detail::frozen341::OutputDisjoint(scope,failure_moved.catalog()->parent(0),1));
    for(const auto* catalog:std::array<const Catalog*,5>{&f.catalog,&copied,&moved,&independent,failure_moved.catalog()})
      CatalogProbes(*catalog,[&](Probe p){SameScope(scope,p);});
    for(std::size_t i=0;i<other.parent_count();++i)for(auto p:Around(other.parent(i),sizeof(fe::ShellFailureParentInput)))SameScope(scope,p);
  }
}
}
