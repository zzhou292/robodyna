#include "SourceAssemblyWallSetupTestSupport.h"
#include <cstring>

namespace crash::cases::source_assembly::test {
namespace sc=tlfea::contact;
TEST(SourceAssemblyWallConfig, ExactFreshDescriptorAndActiveCapDeclarationsPreserveOutputOnFailure) {
    const auto& assembly=WallAssembly();WallInput wall;SourceAssemblyWallSetup setup;
    ASSERT_TRUE(setup.Initialize(assembly,wall.canonical,wall.bytes,WallSettings()));
    auto stamp=DeclaredStamp(assembly);sc::NodalWallDeviceConfig config;
    ASSERT_TRUE(setup.MakeDeviceConfig(stamp,&config));EXPECT_EQ(config.limits.parents,1024u);
    EXPECT_EQ(config.limits.nodes,2048u);EXPECT_EQ(config.limits.global_nodes,2048u);
    EXPECT_EQ(config.owner.rigid_groups.source_instance_id,assembly.source_instance_id());
    EXPECT_EQ(config.owner.rigid_groups.group_count,6u);EXPECT_EQ(config.owner.rigid_groups.member_count,76u);
    SameBits(config.law.wall_x,setup.placed_wall()->geometry()->wall_x());
    SameBits(config.law.stiffness_per_area,setup.certificate()->penalty.stiffness_per_area);
    std::array<unsigned char,sizeof(config)> before{};std::memcpy(before.data(),&config,sizeof(config));
    for(unsigned variant=0;variant<8;++variant) {
        auto bad=stamp;
        if(variant==0)bad.rigid_groups.source_instance_id++;
        if(variant==1)bad.rigid_groups.member_count--;
        if(variant==2)bad.rigid_groups.group_count=0;
        if(variant==3)bad.node_count--;
        if(variant==4)bad.reaction_kick_dt=1;
        if(variant==5)bad.epoch=1;
        if(variant==6)bad.fixed_dt=0;
        if(variant==7)bad.owner_id=0;
        EXPECT_EQ(setup.MakeDeviceConfig(bad,&config).status,SourceAssemblyWallStatus::ScopeMismatch);
        EXPECT_EQ(std::memcmp(before.data(),&config,sizeof(config)),0);
    }
    for(unsigned variant=0;variant<5;++variant) {
        SourceAssemblyWallDeviceLimits limits;
        if(variant==0)limits.counts.parents=914;
        if(variant==1)limits.counts.nodes=1029;
        if(variant==2)limits.counts.global_nodes=1029;
        if(variant==3)limits.max_device_bytes=SIZE_MAX;
        if(variant==4)limits.max_host_bytes=0;
        EXPECT_EQ(setup.MakeDeviceConfig(stamp,&config,limits).status,SourceAssemblyWallStatus::ResourceLimit);
        EXPECT_EQ(std::memcmp(before.data(),&config,sizeof(config)),0);
    }
    const auto stamp_before=stamp;
    EXPECT_EQ(setup.MakeDeviceConfig(stamp,reinterpret_cast<sc::NodalWallDeviceConfig*>(&stamp)).status,SourceAssemblyWallStatus::InvalidInput);
    EXPECT_TRUE(fe::SameRigidGroupInfo(stamp.rigid_groups,stamp_before.rigid_groups));
    EXPECT_EQ(stamp.owner_id,stamp_before.owner_id);EXPECT_TRUE(setup.MakeDeviceConfig(stamp,&config));
}
} // namespace crash::cases::source_assembly::test
