// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include <limits>

namespace type45_model_test {
TEST(Type45Model,ThreeKindsKeepSourceOrderAndDistinctInitialDampingBodyCoefficients) {
  Fixture f;
  auto rows=Inputs(f);
  joint::Model model;
  const auto report=model.Initialize(f.binding,{1,{rows.data(),rows.size()}});
  ASSERT_TRUE(report)<<report.message;
  ASSERT_EQ(model.joints().size(),3);
  EXPECT_TRUE(model.domain()->SharesStorage(f.domain));
  for(unsigned kind=0;kind<3;++kind) {
    const auto& row=model.joints()[kind];
    EXPECT_EQ(row.geometry.source_joint_id,2200512+kind);
    EXPECT_EQ(row.body_groups[0],1); EXPECT_EQ(row.body_groups[1],0);
    EXPECT_EQ(row.domain_nodes[2],kind?f.domain.Find(9307):SIZE_MAX);
    for(unsigned end=0;end<2;++end) {
      const auto& body=f.binding.groups()[row.body_groups[end]];
      const auto j=body.principal.inertia;
      EXPECT_EQ(row.damping[end].mass_kg,body.mass_kg);
      EXPECT_EQ(row.damping[end].mean_principal_inertia_kg_m2,(j.x+j.y+j.z)/3.);
    }
  }
  EXPECT_GT(model.owned_payload_bytes(),f.binding.owned_payload_bytes());
  EXPECT_GT(model.startup_payload_bytes(),model.owned_payload_bytes());
}
TEST(Type45Model,LateWrongBodyKindOrRepeatedJointRejectsWholeModelThenRetrySucceeds) {
  Fixture f; auto rows=Inputs(f); joint::Model model;
  rows.back().body[1].kind=fe::RigidBindingSourceKind::NodalGroup;
  auto report=model.Initialize(f.binding,{1,{rows.data(),rows.size()}});
  EXPECT_EQ(report.status,joint::ModelStatus::SourceMismatch); EXPECT_EQ(report.joint,2);
  EXPECT_FALSE(model.prepared()); EXPECT_EQ(model.joints().size(),0);
  rows=Inputs(f); rows.back().geometry.source_joint_id=rows.front().geometry.source_joint_id;
  report=model.Initialize(f.binding,{1,{rows.data(),rows.size()}});
  EXPECT_EQ(report.status,joint::ModelStatus::DuplicateIdentity); EXPECT_EQ(report.joint,2);
  EXPECT_FALSE(model.prepared());
  rows=Inputs(f); ASSERT_TRUE(model.Initialize(f.binding,{1,{rows.data(),rows.size()}}));
  const auto* retained=model.joints().data();
  EXPECT_EQ(model.Initialize(f.binding,{}).status,joint::ModelStatus::AlreadyInitialized);
  EXPECT_EQ(model.joints().data(),retained);
}
TEST(Type45Model,SourceBitsAxisOnlyAndClosedZeroFreePolicyAreCheckedBeforePublication) {
  Fixture f; auto rows=Inputs(f); joint::Model model;
  rows.back().geometry.position_m[0].x=-0.;
  EXPECT_EQ(model.Initialize(f.binding,{1,{rows.data(),rows.size()}}).status,joint::ModelStatus::SourceMismatch);
  rows=Inputs(f); rows.back().geometry.position_m[2].z=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(model.Initialize(f.binding,{1,{rows.data(),rows.size()}}).status,joint::ModelStatus::InvalidInput);
  rows=Inputs(f); rows.back().property.free_stiffness.translation.x=1;
  EXPECT_EQ(model.Initialize(f.binding,{1,{rows.data(),rows.size()}}).status,joint::ModelStatus::UnsupportedProfile);
  rows=Inputs(f); rows.back().geometry.source_node_id[2]=rows.back().geometry.source_node_id[0];
  EXPECT_EQ(model.Initialize(f.binding,{1,{rows.data(),rows.size()}}).status,joint::ModelStatus::InvalidInput);
  EXPECT_FALSE(model.prepared());
  rows=Inputs(f); ASSERT_TRUE(model.Initialize(f.binding,{1,{rows.data(),rows.size()}}));
  EXPECT_EQ(f.binding.FindMember(model.joints()[2].domain_nodes[2]),nullptr);
}
TEST(Type45Model,ExactCompleteCapAndOverCapBeforeBorrowedInputReads) {
  Fixture f; const auto rows=Inputs(f); joint::Model first;
  ASSERT_TRUE(first.Initialize(f.binding,{1,{rows.data(),rows.size()}}));
  joint::ModelLimits cap; cap.max_host_bytes=first.startup_payload_bytes();
  joint::Model exact;
  ASSERT_TRUE(exact.Initialize(f.binding,{1,{rows.data(),rows.size()}},cap));
  EXPECT_EQ(exact.startup_payload_bytes(),cap.max_host_bytes);
  --cap.max_host_bytes;
  joint::Model retry;
  const joint::JointInput* unreadable=reinterpret_cast<const joint::JointInput*>(0x1000);
  EXPECT_EQ(retry.Initialize(f.binding,{1,{unreadable,rows.size()}},cap).status,joint::ModelStatus::ResourceLimit);
  EXPECT_FALSE(retry.prepared());
  ++cap.max_host_bytes;
  ASSERT_TRUE(retry.Initialize(f.binding,{1,{rows.data(),rows.size()}},cap));
  EXPECT_FALSE(retry.SharesStorage(first));
}
TEST(Type45Model,DeepCopiedRowsAndSharedCompleteRigidBackingOutliveAllBorrowedInputs) {
  auto model=[] {
    Fixture f; auto rows=Inputs(f); joint::Model value;
    const auto report=value.Initialize(f.binding,{1,{rows.data(),rows.size()}});
    EXPECT_TRUE(report)<<report.message;
    rows[0].geometry.source_joint_id=999;
    return value;
  }();
  ASSERT_TRUE(model.prepared());
  auto copy=model;
  EXPECT_TRUE(copy.SharesStorage(model));
  EXPECT_EQ(copy.joints()[0].geometry.source_joint_id,2200512);
  EXPECT_EQ(copy.rigid_binding()->FindMember(copy.joints()[0].domain_nodes[0])->source_node_id,10);
  EXPECT_EQ(copy.rigid_binding()->groups()[copy.joints()[0].body_groups[1]].source_kind,
      fe::RigidBindingSourceKind::Part);
}
} // namespace type45_model_test
