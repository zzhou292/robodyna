// SPDX-License-Identifier: MIT
#include "Fixture.h"

namespace shell_execution_test {
TEST(ShellExecutionPhysical, ExplicitFailureAndPhysicalEntriesRetainTheSameAuthority) {
  Fixture f;
  const auto rows = f.source.Failures();
  fe::ShellBatchFailureBinding failure;
  EXPECT_EQ(failure.Initialize(f.catalog,rows.data(),rows.size()).status,Status::InvalidInput);
  EXPECT_FALSE(failure.prepared());
  ASSERT_EQ(failure.InitializeExecution(f.catalog,rows.data(),rows.size()).status,Status::Success);
  EXPECT_EQ(failure.parent(Family::T3,3)->policy,fe::ShellFailurePolicy::None);
  EXPECT_EQ(failure.parent(Family::Qeph,1)->policy,fe::ShellFailurePolicy::ConstantAllPoints);
  fe::ShellExecutionBinding execution;
  ASSERT_EQ(execution.Initialize(f.catalog,f.ledger,f.rigid).status,Status::Success);
  const fe::ShellFormulationScope scope{&f.shells,&f.catalog,&failure,nullptr};
  EXPECT_EQ(fe::ValidateShellFormulationScope(scope).status,Status::InvalidInput);
  EXPECT_EQ(fe::ValidateShellExecutionScope(scope).status,Status::Success);
  fe::ShellPhysicalBinding physical;
  EXPECT_FALSE(physical.Initialize(scope,f.ledger));
  EXPECT_FALSE(physical.prepared());
  ASSERT_TRUE(physical.InitializeExecution(scope,f.ledger,execution));
  ASSERT_NE(physical.execution(),nullptr);
  EXPECT_TRUE(physical.execution()->Matches(execution));
  EXPECT_TRUE(physical.coefficients()->Matches(*execution.coefficients()));
  EXPECT_EQ(physical.coefficients()->nodes().data(),execution.coefficients()->nodes().data());
  EXPECT_EQ(physical.execution()->parents().data(),execution.parents().data());
  auto copy = physical;
  EXPECT_TRUE(copy.Matches(physical));
  fe::ShellPhysicalBinding limited;
  auto limits = fe::ShellPhysicalBindingLimits{};
  limits.max_host_bytes = physical.owned_payload_bytes()-1;
  EXPECT_EQ(limited.InitializeExecution(scope,f.ledger,execution,limits).status,fe::NodalDomainStatus::ResourceLimit);
  EXPECT_FALSE(limited.prepared());
  limits.max_host_bytes = physical.owned_payload_bytes();
  EXPECT_TRUE(limited.InitializeExecution(scope,f.ledger,execution,limits));
}
TEST(ShellExecutionPhysical, RigidFailureControlsAndStaleExecutionRejectWithoutPublication) {
  Fixture f;
  const auto original = f.source.Failures();
  for (unsigned fault = 0; fault < 3; ++fault) {
    auto rows = original;
    auto& late = rows.back();
    if (fault == 0) {
      late.policy = fe::ShellFailurePolicy::ConstantAllPoints;
      late.constant.failure_strain = 2.5;
    } else if (fault == 1) {
      late.constant.failure_strain = -0.;
    } else {
      ++late.source.source_part_id;
    }
    fe::ShellBatchFailureBinding target;
    const auto result = target.InitializeExecution(f.catalog,rows.data(),rows.size());
    EXPECT_NE(result.status,Status::Success);
    EXPECT_EQ(result.entry,6u);
    EXPECT_FALSE(target.prepared());
    EXPECT_EQ(target.InitializeExecution(f.catalog,original.data(),original.size()).status,Status::Success);
  }
  fe::ShellExecutionBinding empty;
  fe::ShellBatchFailureBinding failure;
  ASSERT_EQ(failure.InitializeExecution(f.catalog,original.data(),original.size()).status,Status::Success);
  fe::ShellPhysicalBinding physical;
  const fe::ShellFormulationScope scope{&f.shells,&f.catalog,&failure,nullptr};
  EXPECT_EQ(physical.InitializeExecution(scope,f.ledger,empty).status,fe::NodalDomainStatus::InvalidInput);
  EXPECT_FALSE(physical.prepared());
  fe::ShellExecutionBinding execution;
  ASSERT_EQ(execution.Initialize(f.catalog,f.ledger,f.rigid).status,Status::Success);
  EXPECT_TRUE(physical.InitializeExecution(scope,f.ledger,execution));
}
TEST(ShellExecutionPhysical, PureQepRigidScopeHasNoInventedFailureOrMaterialPoints) {
  Source source;
  auto q = source.base.geometry.q[0];
  fe::ShellBatchBinding shells;
  ASSERT_EQ(shells.Initialize({&q,nullptr,1,0,4}).status,fe::ShellBindingStatus::Success);
  const fe::ShellPlasticityParentInput parent{Family::Qeph,0,q.source_parent_id,1000,1000,1000};
  const fe::ShellBatchPlasticityBindingInput input{nullptr,&source.base.materials[0],
      &source.base.sections[0],&parent,0,1,1,1};
  fe::ShellBatchPlasticityBinding catalog;
  ASSERT_EQ(catalog.InitializeExecutionCatalog(shells,input).status,Status::Success);
  const fe::ShellFailureParentInput none{parent};
  fe::ShellBatchFailureBinding failure;
  ASSERT_EQ(failure.InitializeExecution(catalog,&none,1).status,Status::Success);
  unsigned points = 99;
  ASSERT_TRUE(catalog.MaterialPointCount(Family::Qeph,0,&points));
  EXPECT_EQ(points,0u);
  EXPECT_EQ(failure.parent_count(),1u);
  EXPECT_EQ(failure.parent(Family::Qeph,0)->policy,fe::ShellFailurePolicy::None);
  EXPECT_EQ(fe::ValidateShellExecutionScope({&shells,&catalog,&failure,nullptr}).status,Status::Success);
}
} // namespace shell_execution_test
