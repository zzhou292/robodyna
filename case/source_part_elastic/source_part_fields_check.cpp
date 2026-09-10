#include "case/SourcePartElasticFields.h"
#include <gtest/gtest.h>
#include <limits>

namespace crash::cases::source_part_elastic {
namespace {
Snapshot Frame(std::uint64_t epoch) {
    Snapshot s;
    s.stamp.owner_id=7;s.stamp.node_count=NodeCount;s.stamp.fixed_dt=0x1p-24;s.stamp.epoch=epoch;
    s.stamp.has_rotations=true;s.stamp.temporal_scheme=tl::fea::NodalTemporalScheme::StaggeredHalfKickStart;
    s.stamp.time=epoch*s.stamp.fixed_dt;
    for(std::size_t n=0;n<NodeCount;++n)s.orientation[4*n]=1;
    if(epoch) {
        s.stamp.velocity_phase=tl::fea::NodalVelocityPhase::PreviousMidpoint;s.stamp.reactions_valid=true;
        s.stamp.reaction_base_epoch=epoch-1;s.stamp.reaction_time=(epoch-1)*s.stamp.fixed_dt;
        s.stamp.velocity_time=s.stamp.reaction_time+.5*s.stamp.fixed_dt;
        s.stamp.reaction_kick_dt=epoch==1?.5*s.stamp.fixed_dt:s.stamp.fixed_dt;
        s.velocity[0]=.25;s.synchronized_velocity[0]=.5;
    }
    return s;
}
TEST(SourcePartFields, InitialFirstSparseAndFinalLikePhasesRemainExplicitAndRaw) {
    for(std::uint64_t epoch:{0u,1u,128u,65536u}) {
        const auto f=Frame(epoch);const auto d=SourcePartFrameFields(f);
        EXPECT_EQ(d["accepted_epoch"].GetUint64(),epoch);
        EXPECT_EQ(output::Bits(d["velocity_time_s"].GetDouble()),output::Bits(f.stamp.velocity_time));
        EXPECT_EQ(output::Bits(d["reaction_kick_dt_s"].GetDouble()),output::Bits(f.stamp.reaction_kick_dt));
        EXPECT_DOUBLE_EQ(d["velocity_xyz_m_per_s"][0].GetDouble(),epoch?.25:0);
        EXPECT_DOUBLE_EQ(d["synchronized_velocity_xyz_m_per_s"][0].GetDouble(),epoch?.5:0);
        EXPECT_STREQ(d["velocity_phase"].GetString(),epoch?"previous_midpoint":"collocated");
    }
}
TEST(SourcePartFields, MalformedPhaseOrRestStartupFailsBeforeEncoding) {
    for(unsigned fault=0;fault<10;++fault) {
        auto f=Frame(fault<4?0:1);
        if(fault==0)f.orientation[0]=0;
        if(fault==1)f.synchronized_velocity[0]=1;
        if(fault==2)f.synchronized_omega[0]=1;
        if(fault==3)f.stamp.velocity_phase=tl::fea::NodalVelocityPhase::PreviousMidpoint;
        if(fault==4)f.stamp.velocity_phase=tl::fea::NodalVelocityPhase::Collocated;
        if(fault==5)f.stamp.reaction_kick_dt=f.stamp.fixed_dt;
        if(fault==6)f.stamp.velocity_time=f.stamp.time;
        if(fault==7)f.stamp.reaction_base_epoch=1;
        if(fault==8)f.stamp.fixed_dt=std::numeric_limits<double>::quiet_NaN();
        if(fault==9)f.stamp.reactions_valid=false;
        EXPECT_THROW(SourcePartFrameFields(f),std::runtime_error)<<fault;
    }
}
} // namespace
} // namespace crash::cases::source_part_elastic
