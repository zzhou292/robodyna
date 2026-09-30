#include "lib_utest/qualification/shell_global_law1_execution/OwnerFixture.h"
#include "../RecordFields.h"
#include "../Buffers.h"
#include "../ParticipantPhase.h"
#include "output/full_shell/tests/TestSupport.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include <numeric>
namespace crash::output::physical_frames::test {
namespace g=global_law1_execution_test;
TEST(GlobalLaw1AcceptedFieldsCuda, RealMixedOwnerCommitDiscardAndRetryRetainZeroPointAvailability) {
    int devices=0;ASSERT_EQ(cudaGetDeviceCount(&devices),cudaSuccess);ASSERT_GT(devices,0);
    g::OwnerFixture physical({g::Thickness::Accepted,.001});
    for(auto& q:physical.source.quads)q.reference.projection_working_length_m=.001;
    ASSERT_NO_THROW(physical.Initialize());
    std::vector<records::ParentPoints> parents;std::vector<ParentField> fields;
    for(std::size_t i=0;i<physical.source.parents.size();++i) {
        const auto& p=physical.source.parents[i];tl::fea::ShellSectionLaw law;unsigned points=99;
        ASSERT_TRUE(physical.physical.catalog.Law(p.family,p.family_index,&law));
        ASSERT_TRUE(physical.physical.catalog.MaterialPointCount(p.family,p.family_index,&points));
        const auto family=Family(p.family);
        parents.push_back({p.source_parent_id,p.source_part_id,2,family,points,Plasticity(law)});
        fields.push_back({family,static_cast<std::uint32_t>(p.family_index),law});
    }
    // Source/mapping identity is a formatting fixture, not a vehicle source
    // export. Owner/configuration/qualification come from the actual publisher.
    auto id=records::test::Id();id.owner=physical.owner.accepted().owner_id;
    id.configuration=g::OwnerFixture::ConfigId;id.qualification=nodal_empty_test::Fixture::Qualification;
    id.source_instance=physical.physical.domain.source_instance_id();
    const auto context=records::Context::Create(id,physical.source.nodes,parents.data(),parents.size(),physical.physical.fixed_dt);
    detail::FrameBuffers buffers(context);std::vector<std::uint32_t> nodes(physical.source.nodes);
    std::iota(nodes.begin(),nodes.end(),0u);
    for(unsigned step=1;step<=3;++step) {
        g::Attempt attempt;
        ASSERT_NO_THROW(physical.Begin(attempt));
        ASSERT_NO_THROW(physical.Prepare(attempt));
        if(step==2) {
            const auto before=physical.Read();physical.Discard();const auto after=physical.Read();
            EXPECT_TRUE(tl::fea::trial_identity::SameStamp(before.stamp,after.stamp));
            EXPECT_EQ(before.x,after.x);EXPECT_EQ(before.v,after.v);
            ASSERT_NO_THROW(physical.Begin(attempt));
            ASSERT_NO_THROW(physical.Prepare(attempt));
        }
        ASSERT_NO_THROW(g::Check(physical.Commit(attempt)));
        auto snapshot=physical.Read();ASSERT_EQ(snapshot.stamp.epoch,step);
        const auto& s=snapshot.stamp;
        const records::FrameStamp stamp{s.epoch,s.reaction_base_epoch,snapshot.common.qeph.attempt,
            s.time,s.reaction_time,s.velocity_time,s.reaction_kick_dt};
        detail::CheckAcceptedParticipant(snapshot.common.qeph,s,stamp,g::OwnerFixture::ConfigId,nodal_empty_test::Fixture::Qualification);
        detail::CheckAcceptedParticipant(snapshot.common.t3,s,stamp,g::OwnerFixture::ConfigId,nodal_empty_test::Fixture::Qualification);
        std::uint8_t qa[3],ta[3];
        for(unsigned i=0;i<3;++i){qa[i]=snapshot.qfailure[i].active;ta[i]=snapshot.tfailure[i].active;}
        detail::StagePositions(nodes,snapshot.x.data(),physical.source.nodes,buffers.Staging());
        detail::StageLayered(context,fields,QephFamily,snapshot.qsections.data(),qa,3,buffers.Staging(),buffers.flags);
        detail::StageLayered(context,fields,T3Family,snapshot.tsections.data(),ta,3,buffers.Staging(),buffers.flags);
        const auto after=physical.Read();ASSERT_TRUE(tl::fea::trial_identity::SameStamp(s,after.stamp));
        buffers.Finish(context,stamp);
        const auto& frame=buffers.frames[buffers.selected];
        EXPECT_EQ(frame.position_xyz,snapshot.x);EXPECT_EQ(parents[0].native_points,0u);EXPECT_EQ(parents[1].native_points,0u);
        EXPECT_EQ(snapshot.qsections[0].elastic(),nullptr);EXPECT_EQ(snapshot.tsections[0].elastic(),nullptr);
        const auto maxima=records::ParentPlasticMaxima(context,frame);EXPECT_FALSE(maxima[0]);EXPECT_FALSE(maxima[1]);
        records::test::Directory dir;
        const auto file=records::WriteFrame(dir.path,"accepted",context,{frame.stamp,frame.position_xyz.data(),
            frame.position_xyz.size(),frame.plastic_points.data(),frame.plastic_points.size()});
        EXPECT_THROW(records::ReadFrame(dir.path,context,file,{}),std::exception);
        const auto loaded=records::ReadFrame(dir.path,context,file,frame.stamp);
        EXPECT_EQ(loaded.position_xyz,frame.position_xyz);EXPECT_EQ(loaded.plastic_points,frame.plastic_points);
    }
}
}
