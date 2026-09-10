#include "VehicleRigidFixture.h"
#include "lib_src/solvers/NodalRigidStorageLayout.h"
#include "lib_src/solvers/NodalStateLayout.h"
#include <cstring>
#include <limits>
namespace vehicle_rigid_test {
namespace nd=fe::nodal_detail;
TEST(VehicleRigidHost,CompleteSourceCountShapesPreserveAllMemberIdentitiesAndNativeMass) {
  for(bool internal:{false,true}) {
    SourceFixture fixture(internal);auto input=fixture.Input();
    auto legacy=input;legacy.limits={};EXPECT_FALSE(fixture.model.Initialize(legacy));
    ASSERT_TRUE(fixture.model.Initialize(input));const auto& model=fixture.model;
    EXPECT_EQ(model.group_count(),internal?673u:759u);EXPECT_EQ(model.member_count(),internal?6170u:7539u);
    EXPECT_EQ(model.members()[model.member_count()-1].global_node,VehicleNodes-1);
    unsigned pairs=0;std::size_t members=0;
    for(std::size_t g=0;g<model.group_count();++g) {
      const auto& p=model.groups()[g];pairs+=p.member_count==2;members+=p.member_count;
      EXPECT_EQ(p.source_group_id,fixture.groups[g].source_group_id);EXPECT_EQ(p.member_offset,members-p.member_count);
      EXPECT_DOUBLE_EQ(p.structural_mass_kg,2*p.member_count);
      double j=0;for(std::size_t m=0;m<p.member_count;++m)j+=.001;
      EXPECT_DOUBLE_EQ(p.native_total_inertia_sum,j);
    }
    EXPECT_EQ(pairs,4u);EXPECT_EQ(members,model.member_count());
    for(std::size_t i=0;i<members;++i) {
      EXPECT_EQ(model.members()[i].source_node_id,fixture.members[i].source_node_id);
      EXPECT_EQ(std::memcmp(&model.members()[i].position,&fixture.members[i].position,sizeof(Vec3)),0);
      EXPECT_DOUBLE_EQ(model.members()[i].total_inertia_kg_m2,.001);
    }
    EXPECT_GT(model.startup_payload_bytes(),model.owned_payload_bytes());
  }
}
TEST(VehicleRigidHost,LastMemberFailureLeavesModelEmptyAndExactRetryPreservesSource) {
  SourceFixture f;const auto original=f.members.back();f.members.back().source_node_id=f.members.front().source_node_id;
  EXPECT_FALSE(f.model.Initialize(f.Input()));EXPECT_FALSE(f.model.prepared());EXPECT_EQ(f.model.members(),nullptr);
  f.members.back()=original;ASSERT_TRUE(f.model.Initialize(f.Input()));
  EXPECT_EQ(f.model.members()[7538].source_node_id,original.source_node_id);
  EXPECT_EQ(f.model.members()[7538].global_node,VehicleNodes-1);
}
TEST(VehicleRigidHost,ExplicitOwnerCountsAndExactPayloadPreflightAreAtomic) {
  constexpr std::size_t object=256; // Explicit object-byte fixture; runtime supplies sizeof(RigidStorage).
  const auto vehicle=fe::NodalRigidOwnerLimits::Vehicle();nd::RigidStorageLayout layout;
  EXPECT_FALSE(layout.Initialize(VehicleNodes,759,7539,{},object));
  ASSERT_TRUE(layout.Initialize(VehicleNodes,759,7539,vehicle,object));
  EXPECT_EQ(layout.groups.offset,0u);EXPECT_EQ(layout.members.offset,759*sizeof(rigid::GroupRange));
  EXPECT_EQ(layout.node_mask.offset,759*sizeof(rigid::GroupRange)+7539*sizeof(rigid::MemberMetric));
  EXPECT_EQ(layout.device_bytes,layout.node_mask.offset+VehicleNodes);
  const auto expected=object+759*(sizeof(fe::NodalRigidGroupProperties)+sizeof(rigid::GroupRange)+sizeof(fe::NodalRigidGroupSnapshot))+
    7539*(sizeof(fe::NodalRigidGroupMember)+sizeof(rigid::MemberMetric))+VehicleNodes;
  EXPECT_EQ(layout.host_bytes,expected);
  const auto before=layout;auto short_host=vehicle;short_host.max_host_bytes=expected-1;
  EXPECT_FALSE(layout.Initialize(VehicleNodes,759,7539,short_host,object));
  EXPECT_EQ(std::memcmp(&layout,&before,sizeof(layout)),0);
  nd::StateLayout whole;
  ASSERT_TRUE(whole.Initialize(VehicleNodes,true,18*759,layout.device_bytes,6*(VehicleNodes+759),128,
    fe::MaxActiveNodalStateDeviceBytes));
  const auto bytes=whole.bytes;
  EXPECT_FALSE(whole.Initialize(VehicleNodes,true,18*759,layout.device_bytes,6*(VehicleNodes+759),128,bytes-1));
  EXPECT_EQ(whole.bytes,bytes);
}
TEST(VehicleRigidHost,MaximumProfileAndLegacyEffectiveMemberLimitRemainDistinct) {
  nd::RigidStorageLayout layout;const auto vehicle=fe::NodalRigidOwnerLimits::Vehicle();
  ASSERT_TRUE(layout.Initialize(20000,1024,8192,vehicle,256));
  EXPECT_FALSE(layout.Initialize(20000,1025,8192,vehicle,256));
  EXPECT_FALSE(layout.Initialize(20000,1024,8193,vehicle,256));
  EXPECT_TRUE(layout.Initialize(20000,64,16384,{},256));
  EXPECT_FALSE(layout.Initialize(20000,64,16385,{},256));
  EXPECT_FALSE(layout.Initialize(20000,1024,2047,vehicle,256));
  std::size_t bytes=123;EXPECT_FALSE(nd::RigidHostPayload(256,SIZE_MAX,1,1,1,1,1,SIZE_MAX,bytes));EXPECT_EQ(bytes,123u);
  for(unsigned fault=0;fault<6;++fault) {auto limits=vehicle;
    if(fault==0)limits.max_groups=0;if(fault==1)limits.max_groups=1025;
    if(fault==2)limits.max_members=0;if(fault==3)limits.max_members=16385;
    if(fault==4)limits.max_host_bytes=0;if(fault==5)limits.max_host_bytes=fe::MaxRigidOwnerHostBytes+1;
    EXPECT_FALSE(layout.Initialize(20000,759,7539,limits,256));
  }
}
} // namespace vehicle_rigid_test
