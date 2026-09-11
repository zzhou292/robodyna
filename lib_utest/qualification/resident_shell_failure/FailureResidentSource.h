#pragma once
#include "../plasticity_binding/PlasticityBindingFixture.h"
#include "lib_src/elements/ShellBatchFailureBinding.h"

namespace resident_failure_test {
namespace fe=tl::fea;
namespace storage=fe::shell_batch_plasticity_detail;
constexpr std::size_t Parents=2,Nodes=5;
// Complete two-Q/two-T source: each family has one LAW1 and one LAW44.
// Failure is optional and source-ordered independently of the family grouping.
struct Source {
  plasticity_binding_test::Fixture seed;
  std::array<fe::ShellQephBindingInput,Parents> q;
  std::array<fe::ShellT3BindingInput,Parents> t;
  std::array<fe::ShellPlasticityMaterialInput,4> materials;
  std::array<fe::ShellPlasticityParentInput,4> parents;
  std::array<fe::ShellFailureParentInput,4> failures;
  Source() {
    q={seed.qeph,seed.qeph};t={seed.t3,seed.t3};q[1].source_parent_id=703;t[1].source_parent_id=704;
    q[1].nodes={0,1,4,3};q[1].reference.position[2]=seed.t3.reference.position[1];
    q[1].reference.node_ids[2]=seed.t3.reference.node_ids[1];
    t[1].nodes={0,4,2};t[1].reference.position[0]=seed.qeph.reference.position[0];
    t[1].reference.node_ids[0]=seed.qeph.reference.node_ids[0];
    materials={seed.materials[0],seed.materials[1],seed.materials[0],seed.materials[1]};
    for(unsigned i=0;i<2;++i){materials[i].law=fe::ShellSectionLaw::LayeredLaw1Nip3;materials[i].curve_id=0;materials[i].rate={};}
    materials[2].material_id=39;materials[2].curve_id=0;
    materials[2].hardening=tl::material::ShellPlasticityHardeningKind::LinearLaw44;
    materials[2].linear={2700,20000};materials[2].rate={true,8000,8,10000};materials[3].material_id=40;
    parents={{{fe::ShellBindingFamily::T3,0,702,82,38,58},
      {fe::ShellBindingFamily::Qeph,0,701,81,37,57},
      {fe::ShellBindingFamily::Qeph,1,703,83,39,57},
      {fe::ShellBindingFamily::T3,1,704,84,40,58}}};
    for(unsigned i=0;i<4;++i) {
      failures[i].source=parents[i];
      if(i>=2){failures[i].policy=fe::ShellFailurePolicy::ConstantAllPoints;failures[i].constant.failure_strain=1.e-6;}
    }
  }
  bool Prepare(fe::ShellBatchBinding& binding,fe::ShellBatchPlasticityBinding& catalog) {
    const auto b=binding.Initialize({q.data(),t.data(),Parents,Parents,Nodes});
    EXPECT_EQ(b.status,fe::ShellBindingStatus::Success)<<b.message;if(b.status!=fe::ShellBindingStatus::Success)return false;
    auto input=seed.catalog();input.curves=seed.curves.data()+1;input.curve_count=1;
    input.materials=materials.data();input.material_count=materials.size();input.parents=parents.data();input.parent_count=parents.size();
    const auto c=catalog.InitializeSections(binding,input);EXPECT_EQ(c.status,fe::ShellPlasticityBindingStatus::Success)<<c.message;
    return c.status==fe::ShellPlasticityBindingStatus::Success;
  }
};
} // namespace resident_failure_test
