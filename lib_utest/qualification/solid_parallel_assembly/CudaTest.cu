#include "CudaFixture.h"
#include <limits>
#include "LaunchFault.h"
#include "../extended_solid_resident/CudaFixture.h"
namespace solid_parallel_test {
class SolidParallelAssemblyCuda:public ::testing::Test {
  void SetUp() override { int count=0;ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess);ASSERT_GT(count,0); }
};
template<class Edit> Snapshot Compare(DevicePacket& device,Edit edit) {
  device.Reset();edit(device);Cuda(device.Launch(true));const auto old=device.Read();
  device.Reset();edit(device);Cuda(device.Launch(false));const auto current=device.Read();
  Same(old,current);return current;
}
TEST_F(SolidParallelAssemblyCuda, FiveFamiliesRepeatedSlotsAndSeedBitsMatchOriginalScatter) {
  for (std::size_t parents:{1u,2u,17u,129u}) {
    SCOPED_TRACE(parents);
    Packet packet(259,parents);DevicePacket device(packet);
    const auto result=Compare(device,[](auto&){});
    EXPECT_EQ(result.control.status,s::BatchStatus::Success);EXPECT_EQ(result.fallback,0u);
    for (unsigned repeat=0;repeat<8;++repeat) {
      const auto next=Compare(device,[](auto&){});EXPECT_EQ(next.fallback,0u);
    }
  }
}
TEST_F(SolidParallelAssemblyCuda, NonRearDuplicateAndChangedValidConnectivityUseOriginalFailureOrFallback) {
  Packet packet;DevicePacket device(packet);
  auto& parent=packet.State().solid24.parents[1];const auto original=parent.domain_nodes[7];
  parent.domain_nodes[7]=parent.domain_nodes[0];packet.Rebuild();device.Upload();
  auto result=Compare(device,[](auto&){});
  EXPECT_EQ(result.control.status,s::BatchStatus::AssemblyFailure);EXPECT_EQ(result.fallback,1u);
  parent.domain_nodes[7]=original;packet.Rebuild();
  // An immutable incidence mismatch cannot silently drop a contribution.
  parent.domain_nodes[7]=(original+20)%packet.nodes;device.Upload();
  result=Compare(device,[](auto&){});EXPECT_EQ(result.control.status,s::BatchStatus::Success);EXPECT_EQ(result.fallback,1u);
  packet.Rebuild();device.Upload();result=Compare(device,[](auto&){});EXPECT_EQ(result.fallback,0u);
}
TEST_F(SolidParallelAssemblyCuda, LateStiffnessAndForceFailuresPreserveOriginalPartialTrialAndFirstError) {
  for (unsigned fault=0;fault<4;++fault) {
    SCOPED_TRACE(fault);
    Packet packet;auto& state=packet.State();
    auto& rear=state.solid18_law44.slab[0][1].cache;
    auto& foam=state.solid18_law90.slab[0][0].cache;
    if (fault==0) rear.rhs_force_n[7].z=std::numeric_limits<double>::infinity();
    if (fault==1) rear.stiffness.translation_n_m=-1;
    if (fault==2) foam.rhs_force_n[3].x=std::numeric_limits<double>::quiet_NaN();
    if (fault==3) {rear.stiffness.translation_n_m=std::numeric_limits<double>::max();foam.rhs_force_n[0].x=std::numeric_limits<double>::infinity();}
    DevicePacket device(packet);auto result=Compare(device,[](auto&){});
    EXPECT_EQ(result.control.status,s::BatchStatus::AssemblyFailure);EXPECT_EQ(result.fallback,1u);
    EXPECT_FALSE(result.records.bounds.valid);
  }
}
TEST_F(SolidParallelAssemblyCuda, AliasedExternalForceViewsRetainSerialRejectionAndOverlapOrder) {
  Packet packet;DevicePacket device(packet);
  auto result=Compare(device,[](auto& d){d.view.forces.force_y=d.view.forces.force_x;});
  EXPECT_EQ(result.control.status,s::BatchStatus::AssemblyFailure);EXPECT_EQ(result.fallback,1u);
  result=Compare(device,[](auto& d){d.view.forces.force_y=d.view.forces.force_x+1;});EXPECT_EQ(result.fallback,1u);
  result=Compare(device,[](auto& d){d.cin.translational_stiffness=d.view.forces.force_x;});EXPECT_EQ(result.fallback,1u);
  result=Compare(device,[](auto&){});EXPECT_EQ(result.control.status,s::BatchStatus::Success);EXPECT_EQ(result.fallback,0u);
}
TEST_F(SolidParallelAssemblyCuda, InvalidPhaseZeroNodeAndLateNonfiniteGeometryKeepValidationPriority) {
  Packet packet;DevicePacket device(packet);
  auto result=Compare(device,[](auto& d){d.view.attempt=2;});EXPECT_EQ(result.control.status,s::BatchStatus::AssemblyFailure);
  result=Compare(device,[](auto& d){d.view.accepted.node_count=0;});EXPECT_EQ(result.control.status,s::BatchStatus::InvalidInput);
  const auto node=packet.State().solid18_law90.parents[1].domain_nodes[7];
  device.seed[3*node]=std::numeric_limits<double>::quiet_NaN();
  result=Compare(device,[](auto&){});EXPECT_EQ(result.control.status,s::BatchStatus::InvalidInput);
  device.seed[3*node]=0;result=Compare(device,[](auto&){});EXPECT_EQ(result.control.status,s::BatchStatus::Success);
}
TEST_F(SolidParallelAssemblyCuda, GeometryFailureWinsOverEarlierArithmeticFailureAndRetry) {
  Packet packet;
  packet.State().solid18.slab[0][0].cache.rhs_force_n[0].x =
      std::numeric_limits<double>::infinity();
  DevicePacket device(packet);
  const auto node = packet.State().solid18_law90.parents[1].domain_nodes[7];
  for (unsigned channel : {0u, 1u}) {
    SCOPED_TRACE(channel);
    const auto offset = channel * 3 * packet.nodes + 3 * node;
    const auto original = device.seed[offset];
    device.seed[offset] = std::numeric_limits<double>::quiet_NaN();
    for (unsigned repeat = 0; repeat < 8; ++repeat) {
      const auto result = Compare(device, [](auto&){});
      EXPECT_EQ(result.control.status, s::BatchStatus::InvalidInput);
      EXPECT_EQ(result.fallback, 1u);
    }
    device.seed[offset] = original;
    const auto retry = Compare(device, [](auto&){});
    EXPECT_EQ(retry.control.status, s::BatchStatus::AssemblyFailure);
    EXPECT_EQ(retry.fallback, 1u);
  }
}
TEST_F(SolidParallelAssemblyCuda, LaunchFaultStopsBeforePublishingOrSerialReplayAndPoisonsPublicBatch) {
  for (unsigned boundary:{1u,2u,3u}) {
    Packet packet;DevicePacket device(packet);
    const auto before=device.Read();
    launch_fault::Arm(boundary);
    EXPECT_EQ(device.Launch(false),cudaErrorLaunchFailure);
    EXPECT_EQ(launch_fault::Observed(),boundary);
    launch_fault::Arm(0);
    const auto after=device.Read();
    EXPECT_EQ(std::memcmp(before.fields.data(),after.fields.data(),before.fields.size()*sizeof(double)),0);
    EXPECT_EQ(after.fallback,0u);
  }
  extended_resident_test::Rig rig;
  ASSERT_TRUE(rig.Initialize());
  fe::NodalTrialToken token;fe::NodalAssemblyView view;
  ASSERT_TRUE(extended_resident_test::Good(rig.owner.BeginTrial(&token,&view)));
  launch_fault::Arm(1);
  const auto failed=rig.batch.AssembleAccepted(rig.owner,token,view);
  launch_fault::Arm(0);
  EXPECT_EQ(failed.status,s::BatchStatus::DeviceFailure);
  EXPECT_EQ(rig.owner.accepted().epoch,0u);
  EXPECT_NE(rig.owner.SealAssembly(token).status,fe::NodalStatus::Ok);
  ASSERT_TRUE(extended_resident_test::Good(rig.owner.BeginTrial(&token,&view)));
  EXPECT_EQ(rig.batch.AssembleAccepted(rig.owner,token,view).status,s::BatchStatus::Unusable);
  rig.owner.Discard();
}
TEST_F(SolidParallelAssemblyCuda, CancellationAndNearOverflowKeepSequentialBitsAcrossFamilies) {
  Packet packet;
  for (std::size_t parent=0;parent<2;++parent) {
    auto& a=packet.State().solid18.slab[0][parent].cache;
    auto& b=packet.State().solid24.slab[0][parent].cache;
    for (unsigned slot=0;slot<8;++slot) {
      a.rhs_force_n[slot]={slot%2 ? -1e16 : 1e16, -0.0, 1.};
      b.rhs_force_n[slot]={slot%2 ? 1. : -1., 0.0, -1.};
    }
  }
  DevicePacket device(packet);
  const auto result=Compare(device,[](auto&){});
  EXPECT_EQ(result.control.status,s::BatchStatus::Success);
  EXPECT_EQ(result.fallback,0u);
}
} // namespace solid_parallel_test
