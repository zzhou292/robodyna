#include "case/SourcePartWallArtifacts.h"
#include "case/source_part_wall/SourcePartWallSetup.h"
#include "case/source_part_elastic/SourcePartElasticPilot.h"
#include "case/source_part_plastic/SourcePartMaterial.h"
#include "case/CanonicalWallArtifacts.h"
#include "output/AcceptedReplay.h"
#include "output/ReplayBundleTestSupport.h"
#include "output/CsvLedgerSegments.h"
#include "output/SourcePartWallArtifactSchema.h"
#include <gtest/gtest.h>
#include <sstream>

namespace crash::cases::source_part_plastic {
namespace {
namespace part=source_part_elastic;namespace out=output;namespace fs=std::filesystem;
std::string SourcePath,WallPath;
class PlasticBundle {
 public:
    fs::path parent,directory;
    explicit PlasticBundle(bool rates=false) {
        const auto pattern=(fs::temp_directory_path()/"source-plastic-output-XXXXXX").string();
        std::vector<char> bytes(pattern.begin(),pattern.end());bytes.push_back(0);const auto made=::mkdtemp(bytes.data());
        out::Require(made,"Cannot create plastic archive test directory");parent=made;directory=parent/"accepted";
        source::SourcePartContactFixture fixture;
        out::Require(source::LoadPinnedSourcePartContact(SourcePath,&fixture).status==source::FixtureStatus::Ok,"Source fixture loading failed");
        const auto wall_bytes=case_data::ReadPinnedWallManifest(WallPath);std::istringstream input(wall_bytes);case_data::CanonicalWall wall;
        out::Require(wall.Load(input).status==case_data::WallStatus::Ok,"Wall fixture loading failed");
        auto config=part::MeshWallConfig(1);
        config.material_model=rates?part::MaterialModel::SourceCowperSymonds:part::MaterialModel::ExperimentalRateIndependentTabulatedJ2;
        if(rates)config.rate={true,8000,8,10000};
        const auto material=LoadPinnedSourcePartMaterial(SourcePath,&config.material);out::Require(bool(material),material.message.c_str());
        config.configuration_id+=100;config.qualification_id+=100;
        part::SourcePartElasticCase run;const auto initialized=run.Initialize(fixture,config,wall,wall_bytes,part::MeshWallSettings(config));
        out::Require(bool(initialized),initialized.message);
        source_part_wall::SourcePartWallArtifacts writer(directory.string(),run,8,2,71,72);writer.WriteFrame(run);
        for(unsigned step=1;step<=4;++step) {
            const auto base=run.owner().accepted();const auto result=run.Step();out::Require(bool(result),result.message);
            writer.RecordInterval(base,run);if(step==1||step%2==0)writer.WriteFrame(run);
        }
        writer.FinishPrefix(run,0,"Accepted plastic startup prefix");
    }
    ~PlasticBundle(){std::error_code error;fs::remove_all(parent,error);}
};
TEST(SourcePartPlasticOutput, ActualAcceptedLayerHistoriesAndPhysicalPlacedMeshReplay) {
  for(bool rates:{false,true}) {
    SCOPED_TRACE(rates);PlasticBundle bundle(rates);out::AcceptedReplay replay;const auto opened=replay.Open(bundle.directory);
    ASSERT_EQ(opened.status,out::ReplayStatus::Ok)<<opened.diagnostic;
    ASSERT_TRUE(replay.info());EXPECT_TRUE(replay.info()->source_plasticity);EXPECT_FALSE(replay.info()->horizon_complete);
    EXPECT_EQ(replay.info()->material_model,rates?"source_cowper_symonds_law44":"experimental_rate_independent_tabulated_j2");
    EXPECT_EQ(replay.info()->frame_count,4u);EXPECT_EQ(replay.info()->triangle_count,182u);
    ASSERT_TRUE(replay.wall());EXPECT_EQ(replay.wall()->GetCoordsVertices().size(),62u);
    out::test_support::ModifiedReplayBundle copy(bundle.directory);
    auto initial=copy.Read(copy.Frame(0));EXPECT_TRUE(initial["contact"].IsNull());
    ASSERT_EQ(initial["plastic_sections"].Size(),94u);
    for(const auto& row:initial["plastic_sections"].GetArray())for(const auto& point:row[12].GetArray())
        for(const auto& number:point.GetArray())EXPECT_EQ(number.GetDouble(),0.);
    EXPECT_EQ(initial["yielded_points"].GetUint64(),0u);EXPECT_EQ(initial["cumulative_plastic_work_J"].GetDouble(),0.);
    auto config=copy.Read("configuration.json");EXPECT_EQ(config["source_rate_effects_active"].GetBool(),rates);
    EXPECT_EQ(config["source_rate_coefficient_per_s"].GetDouble(),8000.);EXPECT_EQ(config["source_yield_stress_Pa"].Size(),46u);
    const auto name=copy.Frame(1);const auto bytes=out::ReadBounded(copy.directory/name,out::SourcePartPlasticWallFieldCap);
    EXPECT_LE(out::test_support::WorstScalarWidth(bytes),out::SourcePartPlasticWallFieldCap);
    const auto plan=out::PlanCsvLedger("accepted-intervals.csv",out::SourcePartWallIntervalHeader,251659,out::SourcePartWallIntervalColumns*26);
    const auto total=plan.total_bytes+1000*out::SourcePartPlasticWallFrameCap+1024*1024;
    EXPECT_GT(total,out::kArtifactTotalCap);EXPECT_LT(total,out::SourcePartPlasticWallTotalCap);
    ASSERT_EQ(replay.Load(3).status,out::ReplayStatus::Ok);EXPECT_EQ(replay.frame()->epoch,4u);
    auto oversized=bytes;oversized.resize(out::SourcePartPlasticWallFieldCap+1,' ');copy.ReplaceBytes(name,oversized);copy.Rehash(name);
    out::AcceptedReplay bad;const auto rejected=bad.Open(copy.directory);EXPECT_EQ(rejected.status,out::ReplayStatus::InvalidBundle);
    EXPECT_NE(rejected.diagnostic.find("schema byte cap"),std::string::npos);
    if(rates) {
        out::test_support::ModifiedReplayBundle changed(bundle.directory);auto wrong=changed.Read("configuration.json");
        wrong["rate_filter_cutoff_hz"].SetDouble(20000);wrong["rate_filter_alpha"].SetDouble(1.);
        changed.Replace("configuration.json",wrong);changed.Rehash("configuration.json");
        out::AcceptedReplay invalid;EXPECT_EQ(invalid.Open(changed.directory).status,out::ReplayStatus::InvalidBundle);
    }
  }
}
TEST(SourcePartPlasticOutput, SourceCurveEpochLateLayerAndSummaryFaultsPreservePublishedReplay) {
    PlasticBundle bundle;
    for(unsigned fault=0;fault<10;++fault) {
        SCOPED_TRACE(fault);out::test_support::ModifiedReplayBundle copy(bundle.directory);out::AcceptedReplay replay;
        ASSERT_EQ(replay.Open(copy.directory).status,out::ReplayStatus::Ok);ASSERT_EQ(replay.Load(1).status,out::ReplayStatus::Ok);
        const auto held=*replay.frame();const auto name=(fault==6||fault==7)?"configuration.json":fault==9?"final-metrics.json":copy.Frame(fault==0?0:1);auto data=copy.Read(name);
        if(fault==0)data["plastic_sections"][93][12][2][5].SetDouble(.001);
        if(fault==1)data["plastic_sections"][93][12][2][5].SetDouble(-.1);
        if(fault==2)data["plastic_sections"][93][1].SetUint64(0);
        if(fault==3)data["plastic_accepted_epoch"].SetUint64(2);
        if(fault==4)data["plastic_sections"][93][12][2][6].SetDouble(1.);
        if(fault==5)data["mean_plastic_strain"].SetDouble(.01);
        if(fault==6)data["source_yield_stress_Pa"][45].SetDouble(363e6);
        if(fault==7)data["source_rate_effects_active"].SetBool(true);
        if(fault==8)data["plastic_sections"][93][11].SetDouble(.003296);
        if(fault==9)data["maximum_plastic_strain"].SetDouble(.01);
        copy.Replace(name,data);copy.Rehash(name);
        EXPECT_EQ(replay.Open(copy.directory).status,out::ReplayStatus::InvalidBundle);
        EXPECT_EQ(replay.frame()->mesh,held.mesh);EXPECT_EQ(replay.frame()->epoch,held.epoch);
    }
}
TEST(SourcePartPlasticOutput, ExplicitInventoryBudgetRejectsOverflowBeforePublishingEntry) {
    PlasticBundle bundle;const auto directory=bundle.parent/"inventory";ASSERT_TRUE(fs::create_directory(directory));
    out::ArtifactInventory inventory(directory,16);out::WriteBytes(directory/"first.txt","123456789012");
    out::WriteBytes(directory/"next.txt","123456789012");inventory.Add("first.txt");EXPECT_EQ(inventory.bytes(),12u);
    EXPECT_THROW(inventory.Add("next.txt"),std::runtime_error);
    EXPECT_EQ(inventory.bytes(),12u);
    EXPECT_NO_THROW(out::ArtifactInventory(directory,out::SourcePartPlasticWallTotalCap));
    EXPECT_THROW(out::ArtifactInventory(directory,out::SourcePartPlasticWallTotalCap+1),std::runtime_error);
}
}
}
int main(int argc,char** argv) {
    if(argc<3)return 2;crash::cases::source_part_plastic::SourcePath=argv[1];crash::cases::source_part_plastic::WallPath=argv[2];
    ::testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();
}
