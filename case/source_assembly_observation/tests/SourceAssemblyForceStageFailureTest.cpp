#include "ForceStageFixture.h"

namespace crash::cases::source_assembly_observation::test {
TEST(SourceAssemblyForceStage,FalseReadbackAssociationsAndFinalSourceGroupNeverPublishValidNumbers) {
    ForceFixture f; f.Later(); ForceStageSummary out; out.replacement=91; const auto saved=Bytes(out);
    for(unsigned kind=0;kind<12;++kind) {
        SCOPED_TRACE(kind); auto input=f.force_input();
        if(kind==0) ++input.before_group_stamp.owner_id;
        if(kind==1) ++input.before_group_stamp.rigid_groups.source_instance_id;
        if(kind==2) input.before_group_stamp.reaction_kick_dt*=2;
        if(kind==3) ++input.frame_prepared.attempt;
        if(kind==4) input.frame_prepared.velocity_time+=H;
        if(kind==5) input.frame_prepared.kinematics.position_xyz=reinterpret_cast<const double*>(0x1234);
        if(kind==6) input.capture_prepared.base_kinematics.angular_velocity_xyz=reinterpret_cast<const double*>(0x5678);
        if(kind==7) ++input.capture_prepared.rigid_groups.member_count;
        if(kind==8) input.capture_prepared.kick_dt*=2;
        if(kind==9) input.capture_prepared.stream=reinterpret_cast<cudaStream_t>(0x4567);
        if(kind==10) {input.base.epoch=UINT64_MAX; input.before_group_stamp=input.base;}
        if(kind==11) {input.base.owner_id++; input.before_group_stamp=input.base;}
        EXPECT_EQ(ObserveForceStage(input,&out).status,Status::WrongIdentity); EXPECT_EQ(Bytes(out),saved);
    }
    for(unsigned kind=0;kind<5;++kind) {
        SCOPED_TRACE(kind); auto input=f.force_input(); auto accelerations=f.group_acceleration;
        auto frames=f.next_groups; auto before=f.old_groups;
        if(kind==0) ++accelerations.back().source_group_id;
        if(kind==1) ++accelerations.back().source_node_set_id;
        if(kind==2) --accelerations.back().member_count;
        if(kind==3) std::swap(frames[4],frames[5]);
        if(kind==4) std::swap(before[4],before[5]);
        input.group_acceleration=accelerations.data(); input.force_groups=frames.data(); input.before_groups=before.data();
        EXPECT_EQ(ObserveForceStage(input,&out).status,Status::WrongIdentity); EXPECT_EQ(Bytes(out),saved);
    }
}
TEST(SourceAssemblyForceStage,MalformedTimesCountsAndIncompleteViewsFailWithoutPublication) {
    ForceFixture f; f.Later(); ForceStageSummary out; out.replacement=52; const auto saved=Bytes(out);
    for(unsigned kind=0;kind<9;++kind) {
        SCOPED_TRACE(kind); auto input=f.force_input();
        if(kind==0) input.prepared.base_time+=H;
        if(kind==1) input.prepared.kick_dt*=2;
        if(kind==2) input.prepared.proposed_time=std::numeric_limits<double>::infinity();
        if(kind==3) {input.base.time=std::numeric_limits<double>::max(); input.before_group_stamp=input.base;}
        if(kind==4) input.prepared.kinematics.position_xyz=nullptr;
        if(kind==5) input.group_count=SIZE_MAX;
        if(kind==6) {input.acceleration_nodes=SIZE_MAX; input.acceleration_xyz=reinterpret_cast<const double*>(1);}
        if(kind==7) {input.acceleration_groups=SIZE_MAX; input.group_acceleration=reinterpret_cast<const fe::NodalRigidGroupAccelerationSnapshot*>(1);}
        if(kind==8) input.before.node_count=0;
        input.frame_prepared=input.capture_prepared=input.prepared;
        EXPECT_FALSE(ObserveForceStage(input,&out)); EXPECT_EQ(Bytes(out),saved);
    }
}
TEST(SourceAssemblyForceStage,EveryInspectedBorrowedRangeAndInputDescriptorRejectOutputOverlap) {
    ForceFixture f; ForceStageSummary out; out.replacement=82; const auto saved=Bytes(out);
    for(unsigned kind=0;kind<10;++kind) {
        SCOPED_TRACE(kind); auto input=f.force_input();
        const auto* alias=reinterpret_cast<const double*>(&out);
        if(kind==0) input.before.velocity_xyz=alias;
        if(kind==1) input.before.angular_velocity_xyz=alias;
        if(kind==2) input.before_groups=reinterpret_cast<const fe::NodalRigidGroupSnapshot*>(&out);
        if(kind==3) input.force_groups=reinterpret_cast<const fe::NodalRigidGroupSnapshot*>(&out);
        if(kind==4) input.acceleration_xyz=alias;
        if(kind==5) input.angular_acceleration_xyz=alias;
        if(kind==6) input.group_acceleration=reinterpret_cast<const fe::NodalRigidGroupAccelerationSnapshot*>(&out);
        if(kind==7) input.acceleration_xyz=reinterpret_cast<const double*>(UINTPTR_MAX-4);
        if(kind==8) input.angular_acceleration_xyz=nullptr;
        if(kind==9) input.group_acceleration=nullptr;
        EXPECT_EQ(ObserveForceStage(input,&out).status,Status::InvalidInput); EXPECT_EQ(Bytes(out),saved);
    }
    alignas(ForceStageSummary) unsigned char overlap[sizeof(ForceStageInput)+sizeof(ForceStageSummary)]{};
    auto* input=new(overlap) ForceStageInput(f.force_input()); const auto bytes=Bytes(overlap);
    EXPECT_EQ(ObserveForceStage(*input,reinterpret_cast<ForceStageSummary*>(overlap)).status,Status::InvalidInput);
    EXPECT_EQ(Bytes(overlap),bytes);
    const auto original=f.old.v;
    EXPECT_EQ(ObserveForceStage(f.force_input(),reinterpret_cast<ForceStageSummary*>(f.old.v.data())).status,Status::InvalidInput);
    EXPECT_EQ(f.old.v,original);
}
TEST(SourceAssemblyForceStage,LateOrdinaryAndGroupNonfiniteValuesAndFiniteOverflowPreserveOutputs) {
    ForceFixture f; f.Later(); ForceStageSummary out; out.replacement=27; const auto saved=Bytes(out);
    for(unsigned kind=0;kind<5;++kind) {
        SCOPED_TRACE(kind); auto input=f.force_input(); auto acceleration=f.acceleration;
        auto group=f.group_acceleration; auto motion=f.old;
        if(kind==0) acceleration.w.back()=std::numeric_limits<double>::quiet_NaN();
        if(kind==1) group.back().angular_acceleration.z=std::numeric_limits<double>::infinity();
        if(kind==2) acceleration.v[3*f.LastOrdinary()]=std::numeric_limits<double>::max();
        if(kind==3) motion.v[3*f.LastOrdinary()]=std::numeric_limits<double>::max();
        if(kind==4) group.back().acceleration.x=std::numeric_limits<double>::max();
        input.before=motion.view(); input.acceleration_xyz=acceleration.v.data(); input.angular_acceleration_xyz=acceleration.w.data();
        input.group_acceleration=group.data();
        EXPECT_FALSE(ObserveForceStage(input,&out)); EXPECT_EQ(Bytes(out),saved);
    }
    ASSERT_TRUE(ObserveForceStage(f.force_input(),&out));
}
} // namespace crash::cases::source_assembly_observation::test
