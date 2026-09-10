#include "SourcePartPlasticPilot.h"
#include "SourcePartPlasticEngineChecks.h"
#include "case/CanonicalWallArtifacts.h"
#include <gtest/gtest.h>
#include <sstream>

namespace crash::cases::source_part_plastic {
namespace {
namespace part=source_part_elastic;
namespace check=engine_check;
std::string SourcePath,WallPath;
struct Fixture {
    source::SourcePartContactFixture source;
    SourcePartMaterial material;
    case_data::CanonicalWall wall;
    std::string wall_bytes;
    Fixture() {
        const auto s=source::LoadPinnedSourcePartContact(SourcePath,&source);
        if(s.status!=source::FixtureStatus::Ok) throw std::runtime_error(s.diagnostic);
        const auto m=LoadPinnedSourcePartMaterial(SourcePath,&material);
        if(!m) throw std::runtime_error(m.message);
        wall_bytes=case_data::ReadPinnedWallManifest(WallPath);
        std::istringstream input(wall_bytes);
        const auto w=wall.Load(input);
        if(w.status!=case_data::WallStatus::Ok) throw std::runtime_error(w.message);
    }
};
TEST(SourcePartPlasticEngine, SourceRateConfigurationAndMovingStartup) {
    Fixture f;const auto config=PlasticWallConfig(f.material,1);
    EXPECT_TRUE(part::ValidConfig(config));EXPECT_TRUE(config.rate.enabled);
    EXPECT_EQ(config.rate.cowper_symonds_c_per_s,8000);EXPECT_EQ(config.rate.cowper_symonds_p,8);
    EXPECT_EQ(config.rate.cutoff_hz,10000);EXPECT_EQ(PlasticWallSettings(config).leading_gap,.020);
    auto invalid=config;invalid.rate.cutoff_hz=1;EXPECT_FALSE(part::ValidConfig(invalid));
    invalid=config;invalid.rate.enabled=false;EXPECT_FALSE(part::ValidConfig(invalid));
    part::SourcePartElasticCase run;
    auto r=run.Initialize(f.source,config,f.wall,f.wall_bytes,PlasticWallSettings(config));ASSERT_TRUE(r)<<r.message;
    part::Snapshot initial;ASSERT_TRUE(run.Capture(&initial));EXPECT_TRUE(initial.plastic.enabled);
    EXPECT_EQ(initial.plastic.maximum_plastic_strain,0);EXPECT_EQ(initial.position,f.source.coordinates());
    SourcePartPlasticState section;ASSERT_TRUE(run.CapturePlasticSectionHistory(&section));
    for(double t:section.qeph_reported_thickness) EXPECT_EQ(t,f.material.declaration().thickness_m);
    EXPECT_GT(run.initial_kinetic_energy(),8);EXPECT_EQ(run.owner().accepted().epoch,0u);
}
TEST(SourcePartPlasticEngine, OriginalMixedPartContactAndRejectedTrialPreserveAllHistories) {
    Fixture f;auto config=PlasticWallConfig(f.material,1);auto settings=PlasticWallSettings(config);
    settings.leading_gap=1e-5; // Short contact integration fixture, not the video's approach distance.
    part::SourcePartElasticCase run;
    const auto init=run.Initialize(f.source,config,f.wall,f.wall_bytes,settings);ASSERT_TRUE(init)<<init.message;
    const auto allocated=run.allocations();
    bool observed_plasticity=false;
    for(unsigned step=1;step<=4096;++step) {
        const auto r=run.Step();ASSERT_TRUE(r)<<r.message<<" measured="<<r.measured<<" limit="<<r.limit;
        if(step%256==0) {
            part::Snapshot observed;ASSERT_TRUE(run.Capture(&observed));
            if(observed.plastic.maximum_plastic_strain>0&&observed.plastic.cumulative_plastic_work_J>0) {
                observed_plasticity=true;break;
            }
        }
    }
    ASSERT_TRUE(observed_plasticity)<<"No nonzero plastic history/work within the declared 4096-step prefix";
    ASSERT_GT(run.wall_metrics()->contact_intervals,0u);
    auto& p=part::SourcePartElasticTestAccess::Internal(run);
    part::Snapshot held;ASSERT_TRUE(run.Capture(&held));
    SourcePartPlasticState held_sections;ASSERT_TRUE(run.CapturePlasticSectionHistory(&held_sections));
    auto held_shells=std::make_unique<part::test::Results>();
    auto observed_shells=std::make_unique<part::test::Results>();
    ASSERT_NO_FATAL_FAILURE(check::ReadShells(run,*held_shells));
    const auto held_contact=std::make_unique<part::wall_contact::NodalWallDeviceResults>(*run.accepted_contact());
    const auto held_metrics=*run.wall_metrics();
    const auto held_q_work=p.accepted_q_work_magnitude,held_t_work=p.accepted_t_work_magnitude;
    const double held_initial_kinetic=run.initial_kinetic_energy();
    ASSERT_TRUE(p.Prepare());ASSERT_TRUE(p.Evaluate());ASSERT_TRUE(p.Observe());
    const auto expected=p.trial;const auto expected_sections=p.trial_plastic;
    const auto expected_shells=std::make_unique<part::test::Results>();
    expected_shells->qeph=p.qresult;expected_shells->t3=p.tresult;
    const auto expected_contact=std::make_unique<part::wall_contact::NodalWallDeviceResults>(p.wall->trial);
    const auto expected_metrics=p.wall->trial_metrics;
    const auto expected_q_work=p.trial_q_work_magnitude,expected_t_work=p.trial_t_work_magnitude;
    p.Discard();
    const auto old_limit=p.config.maximum_displacement;p.config.maximum_displacement=1e-30;
    EXPECT_EQ(run.Step().status,part::Status::EnvelopeFailure);
    p.config.maximum_displacement=old_limit;
    part::Snapshot after;ASSERT_TRUE(run.Capture(&after));
    check::SameSnapshot(held,after);
    // Verify the live owner too; an unchanged host cache alone is insufficient.
    ASSERT_EQ(run.owner().CopyAccepted({after.position.data(),after.velocity.data(),part::NodeCount,
        after.orientation.data(),after.omega.data()},&after.stamp).status,tl::fea::NodalStatus::Ok);
    check::SameSnapshot(held,after);
    SourcePartPlasticState after_sections;ASSERT_TRUE(run.CapturePlasticSectionHistory(&after_sections));
    check::SameSections(after_sections,held_sections);
    ASSERT_NO_FATAL_FAILURE(check::ReadShells(run,*observed_shells));check::SameShells(*held_shells,*observed_shells);
    EXPECT_EQ(source_part_wall::check::Bytes(*held_contact),source_part_wall::check::Bytes(*run.accepted_contact()));
    part::test::SameWallMetrics(held_metrics,*run.wall_metrics());
    EXPECT_EQ(p.accepted_q_work_magnitude,held_q_work);EXPECT_EQ(p.accepted_t_work_magnitude,held_t_work);
    EXPECT_EQ(run.initial_kinetic_energy(),held_initial_kinetic);
    ASSERT_FALSE(::testing::Test::HasFailure());
    ASSERT_TRUE(run.Step());ASSERT_TRUE(run.Capture(&after));ASSERT_TRUE(run.CapturePlasticSectionHistory(&after_sections));
    EXPECT_EQ(after.stamp.epoch,held.stamp.epoch+1);
    check::SameSnapshot(expected,after,true);check::SameSections(after_sections,expected_sections);
    ASSERT_EQ(run.owner().CopyAccepted({after.position.data(),after.velocity.data(),part::NodeCount,
        after.orientation.data(),after.omega.data()},&after.stamp).status,tl::fea::NodalStatus::Ok);
    check::SameSnapshot(expected,after,true);
    ASSERT_NO_FATAL_FAILURE(check::ReadShells(run,*observed_shells));check::SameShells(*expected_shells,*observed_shells);
    source_part_wall::check::SameScientificResult(*expected_contact,*run.accepted_contact());
    part::test::SameWallMetrics(expected_metrics,*run.wall_metrics());
    EXPECT_EQ(p.accepted_q_work_magnitude,expected_q_work);EXPECT_EQ(p.accepted_t_work_magnitude,expected_t_work);
    EXPECT_EQ(run.initial_kinetic_energy(),held_initial_kinetic);
    EXPECT_EQ(run.allocations().device_bytes,allocated.device_bytes);
    EXPECT_EQ(run.allocations().device_allocations,allocated.device_allocations);
}
}
}
int main(int argc,char** argv) {
    if(argc<3)return 2;
    crash::cases::source_part_plastic::SourcePath=argv[1];
    crash::cases::source_part_plastic::WallPath=argv[2];
    ::testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
