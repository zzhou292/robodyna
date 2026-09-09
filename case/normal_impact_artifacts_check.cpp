#include "NormalImpactArtifacts.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/serialization/ChArchiveJSON.h"
#include "chrono_thirdparty/rapidjson/document.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace {
namespace cd=crash::case_data;
namespace fs=std::filesystem;
std::string manifest_path;
class TempRoot {
  public:
    TempRoot() {
        std::string pattern=(fs::temp_directory_path()/"normal-impact-artifacts-XXXXXX").string();
        std::vector<char> buffer(pattern.begin(),pattern.end());buffer.push_back(0);
        const char* directory=::mkdtemp(buffer.data());if(!directory)throw std::runtime_error("Could not create test directory");path=directory;
    }
    ~TempRoot(){std::error_code error;fs::remove_all(path,error);}
    fs::path path;
};
std::string Read(const fs::path& path) {std::ifstream file(path,std::ios::binary);if(!file)throw std::runtime_error("Missing test artifact");return {std::istreambuf_iterator<char>(file),std::istreambuf_iterator<char>()};}
void Write(const fs::path& path,const std::string& text){std::ofstream file(path,std::ios::binary);file<<text;file.close();if(file.fail())throw std::runtime_error("Test fixture write failed");}
void Load(cd::CanonicalWall& wall,const std::string& bytes) {std::istringstream input(bytes);ASSERT_EQ(wall.Load(input).status,cd::WallStatus::Ok);}
rapidjson::Document Json(const fs::path& path) {auto text=Read(path);rapidjson::Document result;result.Parse<rapidjson::kParseFullPrecisionFlag>(text.c_str());EXPECT_FALSE(result.HasParseError());return result;}
std::vector<std::string> Split(const std::string& line) {std::istringstream input(line);std::vector<std::string> fields;std::string field;while(std::getline(input,field,','))fields.push_back(field);return fields;}
std::uint64_t Bits(double value){std::uint64_t bits;std::memcpy(&bits,&value,sizeof(bits));return bits;}

TEST(NormalImpactArtifacts, RequiredPinnedBytesRejectAStillValidJsonMutation) {
    const auto original=cd::ReadPinnedWallManifest(manifest_path);ASSERT_FALSE(original.empty());
    TempRoot root;const auto changed=root.path/"mutated.json";Write(changed,original+"\n");
    EXPECT_THROW(cd::ReadPinnedWallManifest(changed.string()),std::runtime_error);
    EXPECT_THROW(cd::ReadPinnedWallManifest((root.path/"missing.json").string()),std::runtime_error);
}

TEST(NormalImpactArtifacts, ExistingDirectoryAndDifferentBoundGeometryAreRejectedWithoutWrites) {
    const auto bytes=cd::ReadPinnedWallManifest(manifest_path);cd::CanonicalWall wall;Load(wall,bytes);TempRoot root;
    const auto existing=root.path/"existing";ASSERT_TRUE(fs::create_directory(existing));Write(existing/"sentinel","keep existing bytes");
    EXPECT_THROW(cd::NormalImpactArtifacts(existing.string(),bytes,wall,{},.001,1),std::runtime_error);
    EXPECT_EQ(Read(existing/"sentinel"),"keep existing bytes");EXPECT_EQ(std::distance(fs::directory_iterator(existing),fs::directory_iterator{}),1);
    // Schema-valid changes to source IDs do not change coordinates. The writer
    // must still reject this wall paired with the original pinned byte string.
    auto changed=bytes;
    const auto location=changed.find("\"source_segment_set_1001_triangle_ids\"");ASSERT_NE(location,std::string::npos);
    const auto first=changed.find('1',changed.find('[',location));ASSERT_NE(first,std::string::npos);
    // Remove one declared reporting-group member, keeping a valid subset and
    // all geometry identical. It must not be mislabeled as the pinned mapping.
    const auto comma=changed.find(',',first);ASSERT_NE(comma,std::string::npos);changed.erase(first,comma-first+1);
    cd::CanonicalWall altered;Load(altered,changed);
    const auto new_path=root.path/"unrelated-mapping";
    EXPECT_THROW(cd::NormalImpactArtifacts(new_path.string(),bytes,altered,{},.001,1),std::runtime_error);
    EXPECT_FALSE(fs::exists(new_path));
    EXPECT_THROW(cd::NormalImpactArtifacts(new_path.string(),bytes+"\n",wall,{},.001,1),std::runtime_error);
    EXPECT_FALSE(fs::exists(new_path));
}

TEST(NormalImpactArtifacts, ExplicitFailureHasNoSuccessfulManifest) {
    const auto bytes=cd::ReadPinnedWallManifest(manifest_path);cd::CanonicalWall wall;Load(wall,bytes);TempRoot root;
    const auto output=root.path/"failed";cd::NormalImpactArtifacts artifacts(output.string(),bytes,wall,{},.001,1);
    cd::NormalImpactCase uninitialized;EXPECT_THROW(artifacts.Finish(uninitialized,0),std::runtime_error);
    EXPECT_FALSE(fs::exists(output/"manifest.json"));
    artifacts.Fail("Intentional artifact failure",nullptr);ASSERT_TRUE(fs::exists(output/"failure.json"));
    auto failure=Json(output/"failure.json");ASSERT_TRUE(failure.HasMember("status"));EXPECT_STREQ(failure["status"].GetString(),"failed");
    EXPECT_FALSE(fs::exists(output/"manifest.json"));EXPECT_FALSE(fs::exists(output/"final-metrics.json"));
    artifacts.Fail("Second failure cannot overwrite the first",nullptr);
    EXPECT_STREQ(Json(output/"failure.json")["message"].GetString(),"Intentional artifact failure");
}

TEST(NormalImpactArtifacts, TwoActualGpuIntervalsPreservePhaseBitsAndRejectAnotherOwner) {
    const auto bytes=cd::ReadPinnedWallManifest(manifest_path);cd::CanonicalWall wall;Load(wall,bytes);TempRoot root;
    cd::NormalImpactConfig config;config.dt=.0005;const auto output=root.path/"completed";
    cd::NormalImpactArtifacts artifacts(output.string(),bytes,wall,config,.001,1);
    cd::NormalImpactCase run,foreign;ASSERT_EQ(run.Initialize(wall,config).status,cd::ImpactStatus::Ok);
    ASSERT_NE(run.output(),nullptr);ASSERT_NE(run.metrics(),nullptr);
    const auto initial_positions=run.output()->surface().mesh()->GetCoordsVertices();
    artifacts.WriteAcceptedFrame(*run.output(),*run.metrics());
    const auto begin=run.metrics()->stamp;ASSERT_EQ(run.Step().status,cd::ImpactStatus::Ok);ASSERT_NE(run.last_interval(),nullptr);
    artifacts.RecordInterval(begin,*run.metrics(),*run.last_interval());
    EXPECT_THROW(artifacts.WriteAcceptedFrame(*run.output(),*run.metrics()),std::runtime_error); // Old visible epoch cannot label new state.
    ASSERT_EQ(run.Publish().status,crash::visual::Status::Ok);artifacts.WriteAcceptedFrame(*run.output(),*run.metrics());
    EXPECT_THROW(artifacts.Finish(run,0),std::runtime_error);EXPECT_FALSE(fs::exists(output/"manifest.json"));

    ASSERT_EQ(foreign.Initialize(wall,config).status,cd::ImpactStatus::Ok);
    ASSERT_EQ(foreign.Step().status,cd::ImpactStatus::Ok);const auto foreign_begin=foreign.metrics()->stamp;
    ASSERT_EQ(foreign.Step().status,cd::ImpactStatus::Ok);ASSERT_EQ(foreign.Publish().status,crash::visual::Status::Ok);
    EXPECT_THROW(artifacts.RecordInterval(foreign_begin,*foreign.metrics(),*foreign.last_interval()),std::runtime_error);
    EXPECT_THROW(artifacts.WriteAcceptedFrame(*foreign.output(),*foreign.metrics()),std::runtime_error);

    const auto second_begin=run.metrics()->stamp;ASSERT_EQ(run.Step().status,cd::ImpactStatus::Ok);
    artifacts.RecordInterval(second_begin,*run.metrics(),*run.last_interval());ASSERT_EQ(run.Publish().status,crash::visual::Status::Ok);
    artifacts.WriteAcceptedFrame(*run.output(),*run.metrics());
    EXPECT_THROW(artifacts.Finish(foreign,0),std::runtime_error);EXPECT_FALSE(fs::exists(output/"manifest.json"));
    artifacts.Finish(run,0);
    ASSERT_TRUE(fs::exists(output/"manifest.json"));EXPECT_FALSE(fs::exists(output/"failure.json"));
    EXPECT_EQ(cd::ReadPinnedWallManifest((output/"canonical-wall.manifest.json").string()),bytes);
    auto manifest=Json(output/"manifest.json");EXPECT_STREQ(manifest["status"].GetString(),"completed");
    EXPECT_FALSE(manifest["shell_model"].GetBool());EXPECT_FALSE(manifest["vehicle_model"].GetBool());EXPECT_EQ(manifest["accepted_epoch"].GetUint64(),2u);
    ASSERT_TRUE(manifest["artifacts"].IsArray());for(const auto& artifact:manifest["artifacts"].GetArray()) {
        EXPECT_EQ(artifact["sha256"].GetStringLength(),64u);EXPECT_EQ(fs::file_size(output/artifact["file"].GetString()),artifact["bytes"].GetUint64());
    }
    chrono::ChTriangleMeshConnected initial_archive;
    {
        std::ifstream input(output/"accepted-000000.mesh.json");chrono::ChArchiveInJSON archive(input,true);
        archive>>chrono::make_ChNameValue("mesh",initial_archive);
    }
    ASSERT_EQ(initial_archive.GetCoordsVertices().size(),initial_positions.size());
    for(std::size_t i=0;i<initial_positions.size();++i)for(int axis=0;axis<3;++axis)
        EXPECT_EQ(Bits(initial_archive.GetCoordsVertices()[i][axis]),Bits(initial_positions[i][axis]));

    std::istringstream csv(Read(output/"accepted-intervals.csv"));std::string line;ASSERT_TRUE(bool(std::getline(csv,line)));
    const auto headers=Split(line);std::map<std::string,std::size_t> columns;
    for(std::size_t i=0;i<headers.size();++i)columns[headers[i]]=i;
    for(unsigned epoch=1;epoch<=2;++epoch) {
        ASSERT_TRUE(bool(std::getline(csv,line)));const auto row=Split(line);ASSERT_EQ(row.size(),headers.size());
        EXPECT_EQ(std::stoull(row.at(columns.at("base_epoch"))),epoch-1);
        EXPECT_EQ(std::stoull(row.at(columns.at("accepted_epoch"))),epoch);
        EXPECT_DOUBLE_EQ(std::stod(row.at(columns.at("force_eval_time_s"))),(epoch-1)*config.dt);
        EXPECT_DOUBLE_EQ(std::stod(row.at(columns.at("accepted_time_s"))),epoch*config.dt);
        const double force=std::stod(row.at(columns.at("force_surface_x_N_at_tn")));
        EXPECT_NEAR(force,epoch==1?0:-5,1e-10); // F_0=0 at touch; F_1=-K*v0*h for M=1,K=1e4.
    }
    EXPECT_FALSE(bool(std::getline(csv,line)));
    const auto frames=Read(output/"accepted-frames.csv");EXPECT_EQ(std::count(frames.begin(),frames.end(),'\n'),4);
}
}  // namespace

int main(int argc,char** argv) {
    ::testing::InitGoogleTest(&argc,argv);
    if(argc!=2){std::cerr<<"Usage: normal_impact_artifacts_check required-canonical-manifest.json [gtest options]\n";return 2;}
    manifest_path=argv[1];std::ifstream required(manifest_path,std::ios::binary);
    if(!required){std::cerr<<"Required canonical wall asset is missing\n";return 2;}
    return RUN_ALL_TESTS();
}
