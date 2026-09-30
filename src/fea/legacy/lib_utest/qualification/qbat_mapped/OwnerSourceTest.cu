// SPDX-License-Identifier: MIT
#include "OwnerFixture.h"

namespace qbat_mapped_test {
TEST(QbatMappedCuda,FullInitialLedgerWrongRosterAndCoherentForeignMassRejectBeforeAllocation) {
  Fixture f;
  fe::FENodalState owner;
  ASSERT_EQ(InitializeOwner(f,owner).status,fe::NodalStatus::Ok);
  auto config=f.Config();
  config.owner=owner.accepted();
  qb::Batch batch;
  auto witnesses=f.mechanics.witnesses;
  witnesses.back().source_element_id+=1;
  auto source=f.Witnesses();
  source.witnesses=witnesses.data();
  EXPECT_NE(batch.InitializeMapped(config,f.physical,owner,source).status,qb::BatchStatus::Success);
  EXPECT_EQ(batch.allocations().device_allocations,0u);
  fe::ElementMassContributions mass;
  const fe::ElementMassSource rows[]{{18000,777,f.mechanics.domain.Find(777),0},
      {18001,55,f.mechanics.domain.Find(55),.003}};
  ASSERT_TRUE(mass.Initialize(f.mechanics.domain,{1,1000,rows,2}));
  fe::NodalCoefficientLedger foreign;
  ASSERT_TRUE(foreign.InitializeWithSolids({{&f.mechanics.shells,&f.mechanics.springs},&mass,&f.mechanics.solids}));
  fe::ShellPhysicalBinding changed;
  ASSERT_TRUE(changed.Initialize({f.physical.shells(),&f.catalog,&f.failure,nullptr},foreign));
  EXPECT_NE(batch.InitializeMapped(config,changed,owner,f.Witnesses()).status,qb::BatchStatus::Success);
  EXPECT_EQ(batch.allocations().device_allocations,0u);
  ASSERT_EQ(batch.InitializeMapped(config,f.physical,owner,f.Witnesses()).status,qb::BatchStatus::Success);
  EXPECT_EQ(batch.allocations().device_allocations,1u);
}
TEST(QbatMappedCuda,InitialCoordinateFaultAndRoleQueryPreserveRetry) {
  Fixture f;
  fe::FENodalState owner;
  ASSERT_EQ(InitializeOwner(f,owner).status,fe::NodalStatus::Ok);
  const std::size_t part_cin[]{f.mechanics.zero_mass,f.mechanics.cin_model.rows().data[0].secondary_domain_node};
  ASSERT_EQ(owner.ValidateFreeRotationalNodes(part_cin,2).status,fe::NodalStatus::Ok);
  const std::size_t late_absent[]{part_cin[0],f.mechanics.ordinary};
  const auto absent=owner.ValidateFreeRotationalNodes(late_absent,2);
  EXPECT_EQ(absent.status,fe::NodalStatus::InvalidInput);
  EXPECT_EQ(absent.node,f.mechanics.ordinary);
  auto config=f.Config();
  config.owner=owner.accepted();
  fe::NodalTrialToken token;
  fe::NodalAssemblyView view;
  ASSERT_EQ(owner.BeginTrial(&token,&view).status,fe::NodalStatus::Ok);
  const auto node=f.mechanics.domain.Find(55);
  const double original=f.mechanics.x[3*node],changed=original+.001;
  auto* x=const_cast<double*>(view.accepted.position_xyz)+3*node;
  ASSERT_EQ(cudaMemcpyAsync(x,&changed,sizeof(changed),cudaMemcpyHostToDevice,view.stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(view.stream),cudaSuccess);
  qb::Batch batch;
  EXPECT_NE(batch.InitializeMapped(config,f.physical,owner,f.Witnesses()).status,qb::BatchStatus::Success);
  EXPECT_EQ(batch.allocations().device_allocations,0u);
  ASSERT_EQ(cudaMemcpyAsync(x,&original,sizeof(original),cudaMemcpyHostToDevice,view.stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(view.stream),cudaSuccess);
  owner.Discard();
  EXPECT_EQ(batch.InitializeMapped(config,f.physical,owner,f.Witnesses()).status,qb::BatchStatus::Success);
  Fixture fixed;
  const auto last=fixed.mechanics.domain.Find(12);
  fixed.mechanics.rotation_fixed[last]=1;
  fixed.mechanics.ij[last]=0;
  fe::FENodalState constrained;
  ASSERT_EQ(InitializeOwner(fixed,constrained).status,fe::NodalStatus::Ok);
  const std::size_t query[]{fixed.mechanics.zero_mass,last};
  EXPECT_EQ(constrained.ValidateFreeRotationalNodes(query,2).node,last);
  config=fixed.Config();config.owner=constrained.accepted();
  qb::Batch rejected;
  EXPECT_NE(rejected.InitializeMapped(config,fixed.physical,constrained,fixed.Witnesses()).status,qb::BatchStatus::Success);
  EXPECT_EQ(rejected.allocations().device_allocations,0u);
}
} // namespace qbat_mapped_test
