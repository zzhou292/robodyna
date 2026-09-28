// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Type45Fixture.h"
#include <algorithm>
namespace activity_source_test {
using Status=n::TransactionStatus;
a::Controls Controls(){return {a::Deletion::ContainingElement,false,n::startup::SolidErosion::Disabled};}
bool AtNode(a::View v,std::size_t node,std::uint32_t parent) {
  return std::binary_search(v.node_parents.data()+v.node_offsets[node],
      v.node_parents.data()+v.node_offsets[node+1],parent);
}
TEST(NativeActivitySourceType45, ActualMasslessSpringEndpointsContributeButAxisDoesNot) {
  Type45Fixture f;a::Plan plan;const auto source=f.Contact();
  const auto result=plan.Initialize({f.physical,&f.joints},source,Controls());
  ASSERT_EQ(result.status,Status::Ok)<<result.message;const auto view=plan.view();
  EXPECT_EQ(plan.forecast().counts.families[static_cast<std::size_t>(a::Family::Type45)],3u);
  std::size_t observed=0;
  for(std::size_t p=0;p<view.parents.size();++p) {
    const auto& parent=view.parents[p];if(parent.family!=a::Family::Type45)continue;
    const auto& joint=f.joints.joints()[parent.family_index];++observed;
    EXPECT_EQ(parent.source_element_id,joint.geometry.source_joint_id);
    EXPECT_TRUE(AtNode(view,joint.domain_nodes[0],p));EXPECT_TRUE(AtNode(view,joint.domain_nodes[1],p));
    if(joint.domain_nodes[2]!=SIZE_MAX)EXPECT_FALSE(AtNode(view,joint.domain_nodes[2],p));
    for(const auto support:view.containing_parents)EXPECT_NE(support,p);
  }
  EXPECT_EQ(observed,3u);
  a::Plan no_joints;ASSERT_EQ(no_joints.Initialize({f.physical,nullptr},source,Controls()).status,Status::Ok);
  EXPECT_EQ(plan.forecast().counts.parents,no_joints.forecast().counts.parents+3);
  EXPECT_EQ(plan.forecast().counts.incidence,no_joints.forecast().counts.incidence+6);
}
TEST(NativeActivitySourceType45, ModelAndRigidAuthorityMustMatchAndOwnedIdentitySurvivesCopies) {
  Type45Fixture f,foreign;a::Plan plan;const auto source=f.Contact();
  EXPECT_EQ(plan.Initialize({f.physical,&foreign.joints},source,Controls()).status,Status::SourceMismatch);
  EXPECT_FALSE(plan.initialized());
  EXPECT_EQ(plan.Initialize({f.physical,&f.joints},source,Controls()).status,Status::Ok);
  const auto copy=f.joints;EXPECT_TRUE(plan.Matches({f.physical,&copy}));
  EXPECT_FALSE(plan.Matches({f.physical,nullptr}));EXPECT_FALSE(plan.Matches({f.physical,&foreign.joints}));
  fe::type45::Model independently_prepared;
  ASSERT_TRUE(independently_prepared.Initialize(f.rigid.binding,{f.rigid.domain.source_instance_id(),{f.inputs.data(),f.inputs.size()}}));
  EXPECT_FALSE(plan.Matches({f.physical,&independently_prepared}));
}
TEST(NativeActivitySourceType45, AdditionalJointRowsAreChargedBeforePublishing) {
  Type45Fixture f;const auto source=f.Contact();
  const auto with=a::Plan::Preflight({f.physical,&f.joints},source,Controls());
  const auto without=a::Plan::Preflight({f.physical,nullptr},source,Controls());
  ASSERT_EQ(with.report.status,Status::Ok);ASSERT_EQ(without.report.status,Status::Ok);
  EXPECT_GT(with.output_bytes,without.output_bytes);EXPECT_GT(with.startup_bytes,without.startup_bytes);
  a::Limits limits;limits.parents=without.counts.parents;a::Plan plan;
  EXPECT_EQ(plan.Initialize({f.physical,&f.joints},source,Controls(),limits).status,Status::ResourceLimit);
  EXPECT_FALSE(plan.initialized());
}
}
