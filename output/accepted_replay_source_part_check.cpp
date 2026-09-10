#include "AcceptedReplay.h"
#include "AcceptedReplayData.h"
#include "ArtifactIO.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include <gtest/gtest.h>
#include <cstdlib>
#include <fstream>
#include <vector>

namespace crash::output {
namespace {
namespace fs=std::filesystem;
fs::path Source() {
    const char* path=std::getenv("ROBO_DYNA_SOURCE_PART_REPLAY_FIXTURE");
    Require(path&&*path,"ROBO_DYNA_SOURCE_PART_REPLAY_FIXTURE must name the accepted original source-part bundle");return path;
}
class ModifiedBundle {
  public:
    fs::path directory;
    ModifiedBundle() {
        auto pattern=(fs::temp_directory_path()/"source-part-replay-XXXXXX").string();std::vector<char> name(pattern.begin(),pattern.end());name.push_back(0);
        const auto result=::mkdtemp(name.data());Require(result,"Cannot create source replay test directory");directory=result;
        // Share immutable payloads; mutations replace a directory entry before
        // writing. No write ever touches a source inode or original manifest.
        for(const auto& entry:fs::directory_iterator(Source())) if(entry.is_regular_file()) {
            std::error_code error;fs::create_hard_link(entry.path(),directory/entry.path().filename(),error);
            if(error)fs::copy_file(entry.path(),directory/entry.path().filename());
        }
    }
    ~ModifiedBundle(){std::error_code error;fs::remove_all(directory,error);}
    Document Read(const std::string& file)const{return replay_detail::Json(ReadBounded(directory/file,32*1024*1024));}
    void Replace(const std::string& file,const Document& d) {
        fs::remove(directory/file);WriteJson(directory/file,d);
    }
    void Rehash(const std::string& file) {
        auto manifest=Read("manifest.json");const auto bytes=ReadBounded(directory/file,32*1024*1024);bool found=false;
        for(auto& entry:manifest["artifacts"].GetArray())if(file==entry["file"].GetString()) {
            entry["sha256"].SetString(Sha256(bytes).c_str(),manifest.GetAllocator());entry["bytes"].SetUint64(bytes.size());found=true;
        }
        Require(found,"Mutated test file was not inventoried");Replace("manifest.json",manifest);
    }
    std::string Frame(std::uint64_t epoch)const {
        const auto manifest=Read("manifest.json");
        for(const auto& entry:manifest["artifacts"].GetArray()) {
            const std::string name=entry["file"].GetString();
            if(name.size()>12&&name.substr(name.size()-12)==".fields.json") {
                const auto f=Read(name);if(f["accepted_epoch"].GetUint64()==epoch)return name;
            }
        }
        throw std::runtime_error("Required epoch was not saved");
    }
};
TEST(AcceptedReplaySourcePart, OriginalPartStreamsInitialFirstSparseAndFinalAcceptedGeometry) {
    AcceptedReplay reader;auto report=reader.Open(Source());ASSERT_EQ(report.status,ReplayStatus::Ok)<<report.diagnostic;
    ASSERT_EQ(reader.info()->kind,ReplayKind::SourcePartElastic);EXPECT_EQ(reader.info()->node_count,117u);
    EXPECT_EQ(reader.info()->triangle_count,182u);EXPECT_FALSE(reader.wall());ASSERT_GT(reader.info()->frame_count,3u);
    EXPECT_EQ(reader.frame()->epoch,0u);ASSERT_EQ(reader.Load(1).status,ReplayStatus::Ok);EXPECT_EQ(reader.frame()->epoch,1u);
    ASSERT_EQ(reader.Load(reader.info()->frame_count/2).status,ReplayStatus::Ok);EXPECT_GT(reader.frame()->epoch,1u);
    ASSERT_EQ(reader.Load(reader.info()->frame_count-1).status,ReplayStatus::Ok);
    EXPECT_EQ(reader.frame()->epoch,reader.info()->final_epoch);EXPECT_EQ(Bits(reader.frame()->time),Bits(reader.info()->final_time));
}
TEST(AcceptedReplaySourcePart, RehashedRawTimingFaultsRemainInvalid) {
    for(unsigned fault=0;fault<7;++fault) {
        ModifiedBundle b;const auto name=b.Frame(fault==0?0:1);auto fields=b.Read(name);
        if(fault==0)fields["velocity_phase"].SetString("previous_midpoint",fields.GetAllocator());
        if(fault==1)fields["velocity_phase"].SetString("collocated",fields.GetAllocator());
        if(fault==2)fields["reaction_kick_dt_s"].SetDouble(fields["fixed_dt_s"].GetDouble());
        if(fault==3)fields["velocity_time_s"].SetDouble(fields["accepted_time_s"].GetDouble());
        if(fault==4)fields["reaction_base_epoch"].SetUint64(1);
        if(fault==5)fields["t3"]["attempt"].SetUint64(fields["t3"]["attempt"].GetUint64()+1);
        if(fault==6)fields["qeph"]["accepted_force_assembled"].SetBool(false);
        b.Replace(name,fields);b.Rehash(name);AcceptedReplay reader;const auto r=reader.Open(b.directory);
        EXPECT_EQ(r.status,ReplayStatus::InvalidBundle)<<fault<<" "<<r.diagnostic;
    }
}
TEST(AcceptedReplaySourcePart, RehashedSourceMassFamilyAndConnectivityFaultsRemainInvalid) {
    for(unsigned fault=0;fault<6;++fault) {
        ModifiedBundle b;auto c=b.Read("configuration.json");
        if(fault==0)c["reference_nodes"][0]["mass_kg"].SetDouble(0);
        if(fault==1)c["reference_nodes"][0]["source_node_id"].SetUint64(0);
        if(fault==2)c["source_parents"][0]["family"].SetString("T3",c.GetAllocator());
        if(fault==3)c["triangle_binding"][0][5].SetUint64(0);
        if(fault==4)c["source_readiness_sha256"].SetString(std::string(64,'0').c_str(),c.GetAllocator());
        if(fault==5)c["reference_nodes"][0]["reference_xyz_m"][0].SetDouble(c["reference_nodes"][0]["reference_xyz_m"][0].GetDouble()+.001);
        b.Replace("configuration.json",c);b.Rehash("configuration.json");AcceptedReplay reader;
        EXPECT_EQ(reader.Open(b.directory).status,ReplayStatus::InvalidBundle)<<fault;
    }
}
TEST(AcceptedReplaySourcePart, FieldGeometryAndQuaternionCorruptionAreRejected) {
    for(unsigned fault=0;fault<5;++fault) {
        ModifiedBundle b;const auto name=b.Frame(fault>=3?0:1);auto f=b.Read(name);
        if(fault==0)f["position_xyz_m"][0].SetDouble(f["position_xyz_m"][0].GetDouble()+.001);
        if(fault==1)f["orientation_wxyz"][0].SetDouble(2);
        if(fault==2)f["synchronized_velocity_xyz_m_per_s"].PopBack();
        if(fault==3) {f["orientation_wxyz"][0].SetDouble(0);f["orientation_wxyz"][1].SetDouble(1);}
        if(fault==4)f["synchronized_omega_world_xyz_rad_per_s"][0].SetDouble(1);
        b.Replace(name,f);b.Rehash(name);AcceptedReplay reader;EXPECT_EQ(reader.Open(b.directory).status,ReplayStatus::InvalidBundle)<<fault;
    }
}
TEST(AcceptedReplaySourcePart, FailedLoadAndReopenPreservePreviouslyPublishedReader) {
    ModifiedBundle b;AcceptedReplay reader;auto report=reader.Open(b.directory);ASSERT_EQ(report.status,ReplayStatus::Ok)<<report.diagnostic;
    ASSERT_EQ(reader.Load(1).status,ReplayStatus::Ok);const auto old=*reader.frame();const auto old_info=*reader.info();
    const auto name=b.Frame(1);auto fields=b.Read(name);fields["velocity_phase"].SetString("collocated",fields.GetAllocator());
    b.Replace(name,fields);EXPECT_EQ(reader.Load(1).status,ReplayStatus::InvalidFrame);
    EXPECT_EQ(reader.frame()->mesh,old.mesh);EXPECT_EQ(reader.frame()->epoch,old.epoch);
    b.Rehash(name);EXPECT_EQ(reader.Open(b.directory).status,ReplayStatus::InvalidBundle);
    EXPECT_EQ(reader.frame()->mesh,old.mesh);EXPECT_EQ(reader.info()->schema,old_info.schema);
}
} // namespace
} // namespace crash::output
