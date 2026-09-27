// SPDX-License-Identifier: MIT
#pragma once
#include "../qbat_catalog/Fixture.h"
#include "lib_src/assembly/ShellExecutionBinding.h"
#include <vector>
namespace shell_output_range_test {
namespace fe=tl::fea;
using Family=fe::ShellBindingFamily;
using Catalog=fe::ShellBatchPlasticityBinding;
// Repeat the existing six-parent source declarations only. Reference geometry,
// materials, sections, curves and failure policy come from the existing fixture.
struct Collection {
  qbat_catalog_test::Fixture original;
  std::vector<fe::ShellQephBindingInput> q;
  std::vector<fe::ShellT3BindingInput> t;
  std::vector<fe::ShellQbatBindingInput> b;
  std::vector<fe::ShellPlasticityParentInput> parents;
  std::vector<fe::ShellFailureParentInput> failures;
  fe::ShellBatchBinding binding;
  Catalog catalog;
  fe::ShellBatchFailureBinding failure;
  explicit Collection(std::size_t copies=1):q(2*copies),t(3*copies),b(copies) {
    for(std::size_t copy=0;copy<copies;++copy) {
      for(unsigned i=0;i<2;++i){q[2*copy+i]=original.geometry.q[i];q[2*copy+i].source_parent_id=1000+10*copy+i;}
      for(unsigned i=0;i<3;++i){t[3*copy+i]=original.triangles[i];t[3*copy+i].source_parent_id=1002+10*copy+i;}
      b[copy]=original.geometry.b;b[copy].source_parent_id=1005+10*copy;
      for(auto parent:original.parents) {
        if(parent.family==Family::Qeph){parent.family_index+=2*copy;parent.source_parent_id=q[parent.family_index].source_parent_id;}
        else if(parent.family==Family::T3){parent.family_index+=3*copy;parent.source_parent_id=t[parent.family_index].source_parent_id;}
        else {parent.family_index=copy;parent.source_parent_id=b[copy].source_parent_id;}
        parents.push_back(parent);failures.push_back(qbat_catalog_test::Failure(parent));
      }
    }
    EXPECT_EQ(binding.InitializeFormulations({{q.data(),t.data(),q.size(),t.size(),5},b.data(),b.size()}).status,fe::ShellBindingStatus::Success);
    EXPECT_EQ(catalog.InitializeExecutionCatalog(binding,Input()).status,fe::ShellPlasticityBindingStatus::Success);
    EXPECT_EQ(failure.InitializeExecution(catalog,failures.data(),failures.size()).status,fe::ShellPlasticityBindingStatus::Success);
  }
  fe::ShellBatchPlasticityBindingInput Input() const {
    return {&original.curve,original.materials.data(),original.sections.data(),parents.data(),1,3,3,parents.size()};
  }
  fe::ShellFormulationScope Scope() const {return {&binding,&catalog,&failure,nullptr};}
  bool ready() const {return binding.prepared()&&catalog.prepared()&&failure.prepared();}
};
struct Execution {
  fe::NodalNodeDomain domain;
  fe::ShellNodeMap mapping;
  fe::NodalCoefficientLedger ledger;
  fe::NodalRigidAssemblyBinding rigid;
  fe::ShellExecutionBinding value;
  Execution(const fe::ShellBatchBinding& binding,const Catalog& catalog) {
    std::vector<fe::NodalDomainNode> nodes;
    for(const auto& row:binding.active_nodes())nodes.push_back({row.source_id,row.position});
    EXPECT_TRUE(domain.Initialize({77,nodes.data(),nodes.size()}));
    EXPECT_TRUE(mapping.Initialize(binding,domain));EXPECT_TRUE(ledger.Initialize({&mapping,nullptr,nullptr}));
    EXPECT_TRUE(rigid.InitializeEmpty(ledger));
    EXPECT_EQ(value.Initialize(catalog,ledger,rigid).status,fe::ShellPlasticityBindingStatus::Success);
  }
};
}
