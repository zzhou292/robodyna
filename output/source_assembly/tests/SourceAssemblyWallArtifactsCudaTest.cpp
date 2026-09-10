#include "SourceAssemblyWallFieldTestSupport.h"
#include "output/source_assembly/SourceAssemblyWallArtifacts.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include <cuda_runtime_api.h>
#include <cstdlib>

namespace crash::output::assembly::test {
namespace {
struct Directory {
    std::filesystem::path base,path;
    Directory() {std::string pattern=(std::filesystem::temp_directory_path()/"assembly-wall-live-output-XXXXXX").string();
        std::vector<char> chars(pattern.begin(),pattern.end());chars.push_back(0);auto* made=::mkdtemp(chars.data());Require(made,"Output test directory failed");base=made;path=base/"archive";}
    ~Directory() {std::error_code e;std::filesystem::remove_all(base,e);}
};
Document ReadJson(const std::filesystem::path& path) {const auto bytes=ReadBounded(path,kArtifactFileCap);Document d;d.Parse(bytes.data(),bytes.size());Require(!d.HasParseError(),"Invalid test artifact JSON");return d;}
class SourceAssemblyWallArtifactLive:public ::testing::Test {
  protected:
    void SetUp() override {int devices=0;ASSERT_EQ(cudaGetDeviceCount(&devices),cudaSuccess);ASSERT_GT(devices,0);
        const auto r=run.Initialize(prepared::WallAssembly(),PreparedWall(),Configuration());ASSERT_TRUE(r)<<r.message;}
    dynamics::SourceAssemblyWallCase run;
};
}
TEST_F(SourceAssemblyWallArtifactLive, CompleteActualCaseBundlePreservesAllSourceFramesAndIndexedContact) {
    Directory dir;auto r=Request();r.steps=64;r.frame_every=32;SourceAssemblyWallArtifacts writer(dir.path.string(),run,r);
    writer.WriteFrame(run);
    for(unsigned i=0;i<64;++i) {const auto base=run.owner()->accepted();const auto step=run.Step();ASSERT_TRUE(step)<<step.message;
        writer.RecordInterval(base,run);if((i+1)%32==0)writer.WriteFrame(run);}
    writer.Finish(run,0);
    const auto manifest=ReadJson(dir.path/"manifest.json");EXPECT_TRUE(manifest["horizon_complete"].GetBool());EXPECT_EQ(manifest["saved_frames"].GetUint64(),3u);
    EXPECT_EQ(ReadBounded(dir.path/"source-assembly-inventory.json",4*1024*1024),run.bindings()->source().data().authenticated_bytes);
    const auto initial=ReadJson(dir.path/"accepted-000000.fields.json");EXPECT_TRUE(initial["contact"].IsNull());
    const auto final=ReadJson(dir.path/"accepted-000064.fields.json");EXPECT_EQ(final["nodal_fields"]["position_xyz_m"].Size(),3090u);
    EXPECT_EQ(final["sections"]["source_parents"].Size(),915u);EXPECT_EQ(final["contact"]["nodes"].Size(),1030u);
    EXPECT_GT(final["contact"]["resultant_N"][0u].GetDouble(),0);EXPECT_GT(run.diagnostics()->active_contact_nodes,0u);
    const auto metrics=ReadJson(dir.path/"final-metrics.json");EXPECT_EQ(metrics["accepted_epoch"].GetUint64(),64u);
    EXPECT_EQ(ParseCsvLedgerSegments(manifest[kCsvLedgerSegmentsField])[0].interval_count,64u);
}
TEST_F(SourceAssemblyWallArtifactLive, ExplicitStoppedPrefixContainsOnlyActualAcceptedRows) {
    Directory dir;auto r=Request();r.steps=4;r.frame_every=2;SourceAssemblyWallArtifacts writer(dir.path.string(),run,r);writer.WriteFrame(run);
    for(unsigned i=0;i<3;++i) {const auto base=run.owner()->accepted();ASSERT_TRUE(run.Step());writer.RecordInterval(base,run);
        if(i==1||i==2)writer.WriteFrame(run);}
    EXPECT_THROW(writer.Finish(run,0),std::runtime_error);
    EXPECT_THROW(writer.FinishPrefix(run,0,""),std::runtime_error);
    EXPECT_NO_THROW(writer.FinishPrefix(run,0,"explicit bounded output qualification stop"));
    const auto manifest=ReadJson(dir.path/"manifest.json");EXPECT_FALSE(manifest["horizon_complete"].GetBool());
    EXPECT_EQ(manifest["accepted_epoch"].GetUint64(),3u);EXPECT_EQ(manifest["requested_steps"].GetUint64(),4u);
    EXPECT_EQ(ParseCsvLedgerSegments(manifest[kCsvLedgerSegmentsField])[0].interval_count,3u);
}
TEST_F(SourceAssemblyWallArtifactLive, LateCreateOnlyFrameFailurePoisonsWriterWithoutChangingAcceptedMechanics) {
    Directory dir;auto r=Request();r.steps=2;r.frame_every=1;SourceAssemblyWallArtifacts writer(dir.path.string(),run,r);writer.WriteFrame(run);
    const auto base=run.owner()->accepted();ASSERT_TRUE(run.Step());writer.RecordInterval(base,run);const auto accepted=run.owner()->accepted();
    const auto diagnostics=*run.diagnostics();WriteBytes(dir.path/"accepted-000001.fields.json","sentinel\n");
    EXPECT_THROW(writer.WriteFrame(run),std::runtime_error);
    EXPECT_THROW(writer.WriteFrame(run),std::runtime_error);
    EXPECT_THROW(writer.FinishPrefix(run,0,"write failure"),std::runtime_error);writer.Fail("Injected create-only artifact collision");
    EXPECT_TRUE(fe::trial_identity::SameStamp(accepted,run.owner()->accepted()));EXPECT_EQ(run.diagnostics()->cumulative_plastic_work,diagnostics.cumulative_plastic_work);
    EXPECT_FALSE(std::filesystem::exists(dir.path/"manifest.json"));EXPECT_FALSE(std::filesystem::exists(dir.path/"manifest.pending.json"));
    EXPECT_TRUE(std::filesystem::exists(dir.path/"failure.json"));EXPECT_EQ(ReadBounded(dir.path/"accepted-000001.fields.json",64),"sentinel\n");
}
TEST_F(SourceAssemblyWallArtifactLive, ForecastFailureLeavesDirectoryAbsentAndOwnerUnchanged) {
    Directory dir;auto r=Request();r.limits.frames=1;const auto stamp=run.owner()->accepted();const auto allocation=run.allocations();
    EXPECT_THROW(SourceAssemblyWallArtifacts(dir.path.string(),run,r),std::runtime_error);
    EXPECT_FALSE(std::filesystem::exists(dir.path));EXPECT_TRUE(fe::trial_identity::SameStamp(stamp,run.owner()->accepted()));
    EXPECT_EQ(run.allocations().device_bytes,allocation.device_bytes);EXPECT_EQ(run.allocations().device_allocations,allocation.device_allocations);
}
} // namespace crash::output::assembly::test
