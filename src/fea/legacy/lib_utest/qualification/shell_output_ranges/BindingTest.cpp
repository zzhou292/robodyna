// SPDX-License-Identifier: MIT
#include "Probes.h"
namespace shell_output_range_test {
TEST(ShellOutputBinding, EveryProtectedSubobjectMatchesFrozenForSingleAndMixedFamilies) {
  qbat_binding_test::Fixture source;
  for(auto family:{Family::Qeph,Family::T3,Family::Qbat})for(std::size_t count:{1u,129u}) {
    fe::ShellBatchBinding binding;
    std::vector<fe::ShellQephBindingInput> q;
    std::vector<fe::ShellT3BindingInput> t;
    std::vector<fe::ShellQbatBindingInput> b;
    if(family==Family::Qeph) {
      q.assign(count,source.q[0]);for(std::size_t i=0;i<count;++i)q[i].source_parent_id=1000+i;
      ASSERT_EQ(binding.Initialize({q.data(),nullptr,count,0,4},{}).status,fe::ShellBindingStatus::Success);
    } else if(family==Family::T3) {
      t.assign(count,source.t);
      for(std::size_t i=0;i<count;++i){t[i].source_parent_id=1000+i;t[i].nodes={0,1,2};}
      ASSERT_EQ(binding.Initialize({nullptr,t.data(),0,count,3},{}).status,fe::ShellBindingStatus::Success);
    } else {
      b.assign(count,source.b);for(std::size_t i=0;i<count;++i)b[i].source_parent_id=1000+i;
      ASSERT_EQ(binding.InitializeFormulations({{nullptr,nullptr,0,0,4},b.data(),count}).status,fe::ShellBindingStatus::Success);
    }
    BindingProbes(binding,[&](Probe p){SameBinding(binding,p);});
    for(auto p:Extreme())SameBinding(binding,p);
  }
  for(std::size_t copies:{1u,22u,65u}) {
    Collection f(copies);ASSERT_TRUE(f.ready());
    BindingProbes(f.binding,[&](Probe p){SameBinding(f.binding,p);});
  }
}
TEST(ShellOutputBinding, InlineCopiesAndExpandedSharedMovesUseActualRetainedAddresses) {
  for(std::size_t copies:{1u,65u}) {
    Collection f(copies),independent(copies);ASSERT_TRUE(f.ready());ASSERT_TRUE(independent.ready());
    fe::ShellBatchBinding copy(f.binding),moved(std::move(copy));
    ASSERT_TRUE(copy.prepared());ASSERT_TRUE(moved.prepared());
    if(copies==1) EXPECT_NE(&copy.qeph_reference(0),&f.binding.qeph_reference(0));
    else EXPECT_EQ(&copy.qeph_reference(0),&f.binding.qeph_reference(0));
    EXPECT_EQ(&copy.qbat_reference(0),&f.binding.qbat_reference(0));
    EXPECT_NE(&independent.binding.qbat_reference(0),&f.binding.qbat_reference(0));
    BindingProbes(f.binding,[&](Probe p){SameBinding(copy,p);SameBinding(moved,p);SameBinding(independent.binding,p);});
    BindingProbes(moved,[&](Probe p){SameBinding(copy,p);SameBinding(moved,p);});
  }
}
TEST(ShellOutputBinding, HeapRecordGapsRemainAcceptedWhileInlineObjectGapsRemainExcluded) {
  for(std::size_t copies:{1u,65u}) {
    Collection f(copies);ASSERT_TRUE(f.ready());
    const auto check=[&](const void* nodes,std::size_t bytes,const void* next,bool heap) {
      const auto begin=Address(nodes)+bytes,end=Address(next);ASSERT_GT(end,begin);
      const Probe gap{Pointer(begin),static_cast<std::size_t>(end-begin)};
      EXPECT_EQ(fe::shell_formulation_detail::frozen341::BindingOutputDisjoint(f.binding,gap.pointer,gap.bytes),heap);
      SameBinding(f.binding,gap);
      SameBinding(f.binding,{Pointer(begin),0});
      SameBinding(f.binding,{Pointer(begin-1),2});
      SameBinding(f.binding,{Pointer(end-1),2});
    };
    check(f.binding.qeph_nodes(0).data(),sizeof(std::array<std::size_t,4>),&f.binding.qeph_reference(1),copies>1);
    check(f.binding.t3_nodes(0).data(),sizeof(std::array<std::size_t,3>),&f.binding.t3_reference(1),copies>1);
    if(copies>1)check(f.binding.qbat_nodes(0).data(),sizeof(std::array<std::size_t,4>),&f.binding.qbat_reference(1),true);
  }
}
}
