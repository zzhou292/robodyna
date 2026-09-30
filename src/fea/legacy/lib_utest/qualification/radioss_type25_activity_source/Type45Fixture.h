// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../type45_model/Fixture.h"
#include "../qbat_catalog/Fixture.h"
#include "lib_src/collision/radioss_type25/activity_source/Plan.h"
namespace activity_source_test {
namespace a=tlfea::contact::radioss_type25::activity_source;
namespace n=tlfea::contact::radioss_type25;
namespace fe=tl::fea;
struct Type45Fixture {
  type45_model_test::Fixture rigid{false,false,.002,true};
  fe::ShellBatchPlasticityBinding catalog;
  fe::ShellBatchFailureBinding failure;
  fe::ShellExecutionBinding execution;
  fe::ShellPhysicalBinding physical;
  fe::type45::Model joints;
  std::array<fe::type45::JointInput,3> inputs;
  std::array<n::lifecycle::Main,2> mains;
  std::uint64_t parent=100;
  Type45Fixture() {
    qbat_catalog_test::Fixture declaration;
    const fe::ShellPlasticityParentInput parents[]{
      {fe::ShellBindingFamily::Qeph,0,100,1000,1000,1000},
      {fe::ShellBindingFamily::Qeph,1,101,1001,1001,1001},
      {fe::ShellBindingFamily::T3,0,102,2000524,2000524,2000524},
      {fe::ShellBindingFamily::Qbat,0,103,2000524,2000524,2000524}};
    EXPECT_EQ(catalog.InitializeExecutionCatalog(rigid.source.shells,{&declaration.curve,
      declaration.materials.data(),declaration.sections.data(),parents,1,3,3,4}).status,
      fe::ShellPlasticityBindingStatus::Success);
    fe::ShellFailureParentInput policies[4];
    for(unsigned i=0;i<4;++i) {
      policies[i].source=parents[i];policies[i].policy=i<2?fe::ShellFailurePolicy::None:fe::ShellFailurePolicy::ConstantAllPoints;
      if(i>=2)policies[i].constant.failure_strain=2.5;
    }
    EXPECT_EQ(failure.InitializeExecution(catalog,policies,4).status,fe::ShellPlasticityBindingStatus::Success);
    EXPECT_EQ(execution.Initialize(catalog,rigid.ledger,rigid.binding).status,fe::ShellPlasticityBindingStatus::Success);
    const auto bound=physical.InitializeExecution({&rigid.source.shells,&catalog,&failure,nullptr},rigid.ledger,execution);
    EXPECT_TRUE(bound)<<bound.message;
    inputs=type45_model_test::Inputs(rigid);
    EXPECT_TRUE(joints.Initialize(rigid.binding,{rigid.domain.source_instance_id(),{inputs.data(),inputs.size()}}));
    mains[0].global_id=1;mains[0].segment_type=2;mains[1].global_id=2;mains[1].segment_type=-1;
    const auto local=rigid.source.shells.qeph_nodes(0);
    for(unsigned k=0;k<4;++k)mains[0].nodes[k]=static_cast<std::uint32_t>(rigid.shells.owner_index(local[k]));
    constexpr unsigned opposite[]{1,0,3,2};
    for(unsigned k=0;k<4;++k)mains[1].nodes[k]=mains[0].nodes[opposite[k]];
  }
  n::ContactSourceInput Contact() const {
    n::ContactSourceInput source;source.source_id=12;source.topology_generation=1;
    source.primary_main_count=1;source.primary_parent_ids=&parent;
    source.selection.mains=mains.data();source.selection.main_count=mains.size();
    source.selection.node_count=rigid.domain.node_count();source.selection.generation=1;return source;
  }
};
}
