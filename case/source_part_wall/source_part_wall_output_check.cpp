#include "case/SourcePartWallArtifacts.h"
#include "case/source_part_wall/SourcePartWallSetup.h"
#include "case/source_part_elastic/SourcePartElasticPilot.h"
#include "case/CanonicalWallArtifacts.h"
#include "output/AcceptedReplay.h"
#include "output/ReplayBundleTestSupport.h"
#include "output/CsvLedgerSegments.h"
#include "output/SourcePartWallArtifactSchema.h"
#include <gtest/gtest.h>
#include <cstdlib>
#include <sstream>
namespace crash::cases::source_part_wall {
namespace {
namespace part=source_part_elastic;namespace out=output;namespace fs=std::filesystem;
std::string SourcePath,WallPath;
// Bound the actual pretty-printed schema without changing its scientific data.
// Quoted strings (including escaped characters) remain literal; every numeric
// token reserves 26 bytes, covering binary64 and uint64 output, and bools 5.
std::size_t WorstScalarWidth(const std::string& bytes) {
    std::size_t bound=bytes.size();
    for(std::size_t i=0;i<bytes.size();) {
        if(bytes[i]=='"') {
            ++i;
            while(i<bytes.size()&&bytes[i]!='"')i+=bytes[i]=='\\'?2:1;
            out::Require(i<bytes.size(),"Unterminated serialized string");++i;
        } else if(bytes[i]=='-'||(bytes[i]>='0'&&bytes[i]<='9')) {
            const auto first=i++;
            while(i<bytes.size()&&std::string("0123456789.eE+-").find(bytes[i])!=std::string::npos)++i;
            out::Require(i-first<=26,"Numeric token exceeds forecast");bound+=26-(i-first);
        } else if(bytes.compare(i,4,"true")==0) { ++bound;i+=4; }
        else ++i;
    }
    return bound;
}
class BundleFixture {
  public:
    fs::path parent,directory;
    explicit BundleFixture(bool prefix=true) {
        auto pattern=(fs::temp_directory_path()/"source-wall-output-XXXXXX").string();std::vector<char> name(pattern.begin(),pattern.end());name.push_back(0);
        const auto made=::mkdtemp(name.data());out::Require(made,"Cannot create wall output test directory");parent=made;directory=parent/"accepted";
        source::SourcePartContactFixture input;out::Require(source::LoadPinnedSourcePartContact(SourcePath,&input).status==source::FixtureStatus::Ok,"Source fixture loading failed");
        const auto bytes=case_data::ReadPinnedWallManifest(WallPath);std::istringstream stream(bytes);case_data::CanonicalWall canonical;
        out::Require(canonical.Load(stream).status==case_data::WallStatus::Ok,"Wall fixture loading failed");
        part::SourcePartElasticCase run;const auto config=part::MeshWallConfig(1);
        const auto initialized=run.Initialize(input,config,canonical,bytes,part::MeshWallSettings(config));out::Require(bool(initialized),initialized.message);
        SourcePartWallArtifacts writer(directory.string(),run,prefix?8:4,2,41,42);writer.WriteFrame(run);
        for(unsigned step=1;step<=4;++step) {
            const auto base=run.owner().accepted();const auto advanced=run.Step();out::Require(bool(advanced),advanced.message);writer.RecordInterval(base,run);
            if(step==1||step%2==0)writer.WriteFrame(run);
        }
        if(prefix)writer.FinishPrefix(run,0,"Explicit four-interval test prefix");else writer.Finish(run,0);
    }
    ~BundleFixture(){std::error_code error;fs::remove_all(parent,error);}
};
TEST(SourcePartWallOutput, ActualMovingStartupPlacedMeshAndClosedAcceptedPrefixReplay) {
    BundleFixture fixture;out::AcceptedReplay replay;const auto opened=replay.Open(fixture.directory);
    ASSERT_EQ(opened.status,out::ReplayStatus::Ok)<<opened.diagnostic;ASSERT_TRUE(replay.info());ASSERT_TRUE(replay.wall());
    EXPECT_EQ(replay.info()->kind,out::ReplayKind::SourcePartWall);EXPECT_FALSE(replay.info()->horizon_complete);
    EXPECT_EQ(replay.info()->stop_reason,"Explicit four-interval test prefix");EXPECT_EQ(replay.info()->final_epoch,4u);
    EXPECT_EQ(replay.info()->frame_count,4u);EXPECT_EQ(replay.info()->node_count,117u);EXPECT_EQ(replay.info()->triangle_count,182u);
    EXPECT_EQ(replay.wall()->GetCoordsVertices().size(),62u);EXPECT_EQ(replay.wall()->GetIndicesVertices().size(),100u);
    out::test_support::ModifiedReplayBundle copy(fixture.directory);const auto initial=copy.Read(copy.Frame(0));const auto config=copy.Read("configuration.json");
    EXPECT_TRUE(initial["contact"].IsNull());EXPECT_EQ(std::string(initial["contact_state"].GetString()),"certified_separated_startup");
    for(unsigned n=0;n<117;++n) {
        EXPECT_EQ(initial["velocity_xyz_m_per_s"][3*n].GetDouble(),1);EXPECT_EQ(initial["synchronized_velocity_xyz_m_per_s"][3*n].GetDouble(),1);
    }
    EXPECT_GT(config["initial_kinetic_J"].GetDouble(),0);const auto placement=copy.Read("placed-wall-placement.json");
    const double wall_x=placement["represented_wall_x_m"].GetDouble();EXPECT_NE(wall_x,.05);
    for(const auto& x:replay.wall()->GetCoordsVertices())EXPECT_EQ(out::Bits(x.x()),out::Bits(wall_x));
    ASSERT_EQ(replay.Load(1).status,out::ReplayStatus::Ok);EXPECT_EQ(replay.frame()->epoch,1u);
    const auto first=copy.Read(copy.Frame(1));EXPECT_EQ(std::string(first["contact"]["phase"].GetString()),"prepared_candidate_of_committed_interval");
    EXPECT_EQ(first["reaction_kick_dt_s"].GetDouble(),part::PilotStep/2);EXPECT_EQ(first["contact"]["parents"].Size(),94u);
    const auto field_bytes=out::ReadBounded(copy.directory/copy.Frame(1),out::SourcePartWallFieldCap);
    EXPECT_GT(field_bytes.size(),256u*1024); // Reproduces the original underestimated dynamic-frame cap.
    EXPECT_LE(WorstScalarWidth(field_bytes),out::SourcePartWallFieldCap);
    const auto mesh_bytes=out::ReadBounded(copy.directory/"accepted-000001.mesh.json",out::SourcePartWallMeshCap);
    EXPECT_LE(WorstScalarWidth(mesh_bytes),out::SourcePartWallMeshCap);
    EXPECT_LE(117u*83u+182u*14u,out::SourcePartWallObjCap);
    EXPECT_LE(fs::file_size(copy.directory/"accepted-000001.obj"),out::SourcePartWallObjCap);
    const auto full=out::PlanCsvLedger("accepted-intervals.csv",out::SourcePartWallIntervalHeader,
        131072,out::SourcePartWallIntervalColumns*26);
    EXPECT_LT(full.total_bytes+259*out::SourcePartWallFrameCap+1024*1024,out::kArtifactTotalCap);
    ASSERT_EQ(replay.Load(3).status,out::ReplayStatus::Ok);EXPECT_EQ(replay.frame()->epoch,4u);
    const auto manifest=copy.Read("manifest.json");EXPECT_EQ(manifest["requested_steps"].GetUint64(),8u);
    EXPECT_EQ(config[out::kCsvLedgerSegmentsField][0u]["interval_count"].GetUint64(),8u);
    EXPECT_EQ(manifest[out::kCsvLedgerSegmentsField][0u]["interval_count"].GetUint64(),4u);
    std::string oversized=field_bytes;oversized.resize(out::SourcePartWallFieldCap+1,' ');
    copy.ReplaceBytes(copy.Frame(1),oversized);copy.Rehash(copy.Frame(1));out::AcceptedReplay rejected;
    const auto rejection=rejected.Open(copy.directory);
    EXPECT_EQ(rejection.status,out::ReplayStatus::InvalidBundle);
    EXPECT_NE(rejection.diagnostic.find("schema byte cap"),std::string::npos);
}
TEST(SourcePartWallOutput, RehashedStartupMidpointCandidateAndEnergyFaultsAreRejected) {
    BundleFixture fixture;
    for(unsigned fault=0;fault<7;++fault) {
        SCOPED_TRACE(fault);out::test_support::ModifiedReplayBundle b(fixture.directory);const auto name=b.Frame(fault<2?0:1);auto f=b.Read(name);
        if(fault==0)f["velocity_xyz_m_per_s"][0].SetDouble(0);
        if(fault==1){f["contact"].SetObject();f["contact"].AddMember("phase","prepared_candidate_of_committed_interval",f.GetAllocator());}
        if(fault==2)f["velocity_time_s"].SetDouble(f["accepted_time_s"].GetDouble());
        if(fault==3)f["contact"]["attempt"].SetUint64(f["contact"]["attempt"].GetUint64()+1);
        if(fault==4)f["contact"]["parents"][0u]["source_element_id"].SetUint64(0);
        if(fault==5)f["contact"]["nodes"][116]["wall_face"].SetUint64(UINT64_MAX);
        if(fault==6)f["energy_allowance_J"].SetDouble(f["energy_allowance_J"].GetDouble()+1);
        b.Replace(name,f);b.Rehash(name);out::AcceptedReplay replay;EXPECT_EQ(replay.Open(b.directory).status,out::ReplayStatus::InvalidBundle);
    }
}
TEST(SourcePartWallOutput, RehashedPlacementNativeEnergyAndCompletionClaimsRemainBoundToActualData) {
    BundleFixture fixture;
    for(unsigned fault=0;fault<7;++fault) {
        SCOPED_TRACE(fault);out::test_support::ModifiedReplayBundle b(fixture.directory);
        const std::string name=fault<2?"placed-wall-placement.json":fault<4||fault==6?"configuration.json":fault==5?"placed-wall.mesh.json":"manifest.json";auto d=b.Read(name);
        if(fault==0)d["declared_translation_x_binary64"].SetUint64(d["declared_translation_x_binary64"].GetUint64()+1);
        if(fault==1)d["triangle_identity"][99][1].SetUint64(0);
        if(fault==2)d["initial_kinetic_J"].SetDouble(d["initial_kinetic_J"].GetDouble()+1);
        if(fault==3)d["reference_nodes"][116]["mass_kg"].SetDouble(2*d["reference_nodes"][116]["mass_kg"].GetDouble());
        if(fault==4)d["horizon_complete"].SetBool(true);
        if(fault==5)d["mesh"]["m_vertices"][0u]["x"].SetDouble(d["mesh"]["m_vertices"][0u]["x"].GetDouble()+.001);
        if(fault==6)d["requested_horizon_s"].SetDouble(2*d["requested_horizon_s"].GetDouble());
        b.Replace(name,d);if(name!="manifest.json")b.Rehash(name);out::AcceptedReplay replay;
        if(fault==5) {
            auto placement=b.Read("placed-wall-placement.json");const auto hash=out::Sha256(out::ReadBounded(b.directory/name,1024*1024));
            placement["placed_mesh_sha256"].SetString(hash.c_str(),placement.GetAllocator());b.Replace("placed-wall-placement.json",placement);b.Rehash("placed-wall-placement.json");
        }
        EXPECT_EQ(replay.Open(b.directory).status,out::ReplayStatus::InvalidBundle);
    }
}
TEST(SourcePartWallOutput, UnsavedAcceptedIntervalsCannotRelaxTheFixedEnergyBudget) {
    BundleFixture fixture;
    for(unsigned column:{28u,32u}) {
        out::test_support::ModifiedReplayBundle b(fixture.directory);std::istringstream input(out::ReadBounded(b.directory/"accepted-intervals.csv",1024*1024));
        std::ostringstream changed;std::string line;unsigned row_index=0;
        while(std::getline(input,line)) {
            if(row_index++==3) { // Epoch 3 is deliberately absent from the saved-frame index.
                std::istringstream row(line);std::vector<std::string> values;std::string value;
                while(std::getline(row,value,','))values.push_back(value);ASSERT_EQ(values.size(),34u);
                values[column]="1";line.clear();for(unsigned j=0;j<values.size();++j){if(j)line+=',';line+=values[j];}
            }
            changed<<line<<'\n';
        }
        b.ReplaceBytes("accepted-intervals.csv",changed.str());b.Rehash("accepted-intervals.csv");out::AcceptedReplay replay;
        EXPECT_EQ(replay.Open(b.directory).status,out::ReplayStatus::InvalidBundle);
    }
}
TEST(SourcePartWallOutput, CompleteHorizonAndFailedReaderLoadKeepTheirPublishedState) {
    BundleFixture fixture(false);out::test_support::ModifiedReplayBundle b(fixture.directory);out::AcceptedReplay replay;
    const auto opened=replay.Open(b.directory);ASSERT_EQ(opened.status,out::ReplayStatus::Ok)<<opened.diagnostic;
    EXPECT_TRUE(replay.info()->horizon_complete);EXPECT_TRUE(replay.info()->stop_reason.empty());ASSERT_EQ(replay.Load(1).status,out::ReplayStatus::Ok);
    const auto previous=*replay.frame();const auto wall=replay.wall();const auto name=b.Frame(1);auto fields=b.Read(name);
    fields["contact"]["base_epoch"].SetUint64(8);b.Replace(name,fields);
    EXPECT_EQ(replay.Load(1).status,out::ReplayStatus::InvalidFrame);EXPECT_EQ(replay.frame()->mesh,previous.mesh);EXPECT_EQ(replay.wall(),wall);
    b.Rehash(name);EXPECT_EQ(replay.Open(b.directory).status,out::ReplayStatus::InvalidBundle);
    EXPECT_EQ(replay.frame()->epoch,previous.epoch);EXPECT_EQ(replay.frame()->mesh,previous.mesh);EXPECT_EQ(replay.wall(),wall);
}
}
}
int main(int argc,char** argv) {
    if(argc<3)return 2;crash::cases::source_part_wall::SourcePath=argv[1];crash::cases::source_part_wall::WallPath=argv[2];
    ::testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();
}
