#include "Fixture.h"
#include "CaptureAssertions.h"
#include "../Internal.h"
#include "output/physical_run/ViewerInput.h"
#include <gtest/gtest.h>
#include <array>
namespace crash::cases::native_scene::test {
namespace {
using capture_test::Copy;using capture_test::Same;using capture_test::SameHistory;

}
TEST(NativeSceneDynamicsCuda, GeneratedInitialStateDiscardRetryAndCommonCaptureStayAtomic) {
    SourceFixture source;auto dynamics=source.MakeDynamics();auto capture=source.MakeCapture(dynamics);
    EXPECT_FALSE(capture->native_state().available);capture->Capture();const auto initial=capture->native_state();
    ASSERT_TRUE(initial.available);ASSERT_EQ(initial.stamp.epoch,0u);EXPECT_FALSE(initial.stamp.reactions_valid);
    EXPECT_EQ(initial.numerical_mass_kg,0);const auto& ledger=*source.physical.physical().coefficients();
    for(std::size_t i=0;i<initial.nodes;++i) {
        EXPECT_EQ(output::Bits(initial.mass_kg[i]),output::Bits(ledger.nodes()[i].coefficients.mass));
        EXPECT_EQ(output::Bits(initial.inertia_kg_m2[i]),output::Bits(ledger.nodes()[i].coefficients.isotropic_inertia));
        EXPECT_EQ(initial.orientation_wxyz[4*i],1.);
        const auto velocity=tl::fea::shell_startup_detail::ProjectVelocity(source.physical.startup().uniform_velocity,source.physical.translation_fixed_bits()[i]);
        EXPECT_EQ(output::Bits(initial.velocity_xyz[3*i]),output::Bits(velocity.x));
        EXPECT_EQ(output::Bits(initial.velocity_xyz[3*i+1]),output::Bits(velocity.y));EXPECT_EQ(output::Bits(initial.velocity_xyz[3*i+2]),output::Bits(velocity.z));
        for(unsigned j=0;j<3;++j)EXPECT_EQ(initial.spin_xyz[3*i+j],0.);
    }
    const auto before=Copy(initial);const auto saved_history=std::vector<native::NativeGeometryHistory>(capture->native_history(),capture->native_history()+capture->secondary_count());
    const auto flags=std::vector<int>(capture->initial_contact_flags(),capture->initial_contact_flags()+capture->secondary_count());
    dynamics.PrepareStep();
    EXPECT_THROW(dynamics.PrepareStep(),std::exception);
    EXPECT_TRUE(dynamics.has_prepared_step());EXPECT_EQ(dynamics.accepted().epoch,0u);capture->Capture();Same(before,capture->native_state());
    dynamics.DiscardStep();EXPECT_FALSE(dynamics.has_prepared_step());capture->Capture();Same(before,capture->native_state());
    for(std::size_t i=0;i<flags.size();++i){EXPECT_EQ(capture->initial_contact_flags()[i],flags[i]);
        SameHistory(capture->native_history()[i],saved_history[i]);}
    dynamics.PrepareStep();
    EXPECT_THROW(dynamics.PrepareStep(),std::exception);
    EXPECT_TRUE(dynamics.has_prepared_step());dynamics.CommitStep();capture->Capture();
    EXPECT_EQ(capture->native_state().stamp.epoch,1u);EXPECT_TRUE(capture->native_state().stamp.reactions_valid);
    EXPECT_EQ(capture->frame()->stamp.epoch,1u);EXPECT_EQ(capture->activity()->stamp().epoch,1u);
    EXPECT_EQ(capture->Interval().values().native_contact->publication_generation,1u);
    auto wrong=source.identity();wrong.configuration=999;EXPECT_THROW(dynamics.MakeCapture(source.archive.mapping(),wrong),std::exception);
}
TEST(NativeSceneDynamicsCuda, DestructionRevokesPendingTrialBeforeParticipantsThenFreshOwnerRetries) {
    SourceFixture source;
    {auto pending=source.MakeDynamics();pending.PrepareStep();EXPECT_TRUE(pending.has_prepared_step());}
    auto retry=source.MakeDynamics();retry.PrepareStep();retry.CommitStep();auto capture=source.MakeCapture(retry);capture->Capture();
    EXPECT_EQ(capture->native_state().stamp.epoch,1u);
}
TEST(NativeSceneDynamicsCuda, ActualControllerClosesCompleteAndStoppedPrefixWithPortableArchive) {
    SourceFixture source;
    for(bool prefix:{false,true}) {
        RunConfig config;config.dynamics=source.config();config.run_id=prefix?907:906;config.steps=4;config.samples=3;
        const auto run=PreparedNativeSceneRun::Prepare(source.contact,source.archive,config);ft::Directory directory;
        vehicle_run::Control control;if(prefix)control.maximum_accepted_intervals=2;
        const auto result=run.Execute(directory.path,control);
        ASSERT_TRUE(result.loop.valid_manifest);ASSERT_TRUE(result.manifest);ASSERT_TRUE(result.viewer_input);ASSERT_TRUE(result.summary);
        EXPECT_TRUE(result.output_error.empty());EXPECT_EQ(result.loop.progress.accepted.epoch,prefix?2u:4u);
        const auto input=output::physical_run::ReadViewerInput(directory.path,*result.viewer_input);
        const auto replay=output::physical_run::Replay::Open(output::physical_run::ViewerArchivePath(directory.path,input),input.manifest,input.source,input.mapping_sha256);
        EXPECT_EQ(replay.index().accepted_intervals,prefix?2u:4u);EXPECT_EQ(replay.index().horizon_complete,!prefix);
        EXPECT_EQ(replay.index().frames.front().stamp.epoch,0u);EXPECT_EQ(replay.index().frames.back().stamp.epoch,prefix?2u:4u);
    }
}
}
