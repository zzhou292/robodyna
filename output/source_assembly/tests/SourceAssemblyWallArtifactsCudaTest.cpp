#include "SourceAssemblyWallFieldTestSupport.h"
#include "output/source_assembly/SourceAssemblyWallArtifacts.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include "case/source_assembly_dynamics/tests/Fixture.h"
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
Document ReadJson(const std::filesystem::path& path) {const auto bytes=ReadBounded(path,kArtifactFileCap);Document d;d.Parse<rapidjson::kParseFullPrecisionFlag>(bytes.data(),bytes.size());Require(!d.HasParseError(),"Invalid test artifact JSON");return d;}
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
TEST_F(SourceAssemblyWallArtifactLive, RejectedNextAttemptCanArchiveTheNewerUnshownAcceptedPrefix) {
    using Access=cases::source_assembly_dynamics::SourceAssemblyDynamicsTestAccess;
    Directory dir;auto r=Request();r.steps=4;r.frame_every=2;
    SourceAssemblyWallArtifacts writer(dir.path.string(),run,r);writer.WriteFrame(run); // Visible/archive epoch0 only.
    const auto base=run.owner()->accepted();ASSERT_TRUE(run.Step());writer.RecordInterval(base,run);
    const auto before=Access::Accepted(run);const auto stamp=run.owner()->accepted();ASSERT_EQ(stamp.epoch,1u);
    const auto allocation=run.allocations();const auto host=run.host_payload_bytes();
    const auto rejected=Access::RejectLate(run,Access::Fault::LastWallFace); // Both material histories and owner candidate2 were prepared.
    ASSERT_EQ(rejected.status,dynamics::Status::ComponentFailure)<<rejected.message;
    auto exact=[](const auto& a,const auto& b) {
        ASSERT_EQ(a.size(),b.size());EXPECT_EQ(std::memcmp(a.data(),b.data(),a.size()*sizeof(a[0])),0);
    };
    auto unchanged=[&] {
        const auto& now=Access::Accepted(run);EXPECT_TRUE(fe::trial_identity::SameStamp(stamp,run.owner()->accepted()));
        exact(before.fields.x,now.fields.x);exact(before.fields.v,now.fields.v);exact(before.fields.w,now.fields.w);
        exact(before.fields.orientation,now.fields.orientation);exact(before.fields.reaction,now.fields.reaction);
        exact(before.fields.couple,now.fields.couple);exact(before.fields.groups,now.fields.groups);
        exact(before.parents.qeph,now.parents.qeph);exact(before.parents.t3,now.parents.t3);
        exact(before.parents.qsection,now.parents.qsection);exact(before.parents.tsection,now.parents.tsection);
        exact(before.wall.parents,now.wall.parents);exact(before.wall.nodes,now.wall.nodes);exact(before.wall.wall_face,now.wall.wall_face);
        EXPECT_EQ(std::memcmp(&before.diagnostics,&now.diagnostics,sizeof(before.diagnostics)),0);
        EXPECT_EQ(std::memcmp(&before.wall.diagnostics,&now.wall.diagnostics,sizeof(before.wall.diagnostics)),0);
        EXPECT_EQ(before.qwork_magnitude,now.qwork_magnitude);EXPECT_EQ(before.twork_magnitude,now.twork_magnitude);
        EXPECT_EQ(run.allocations().device_bytes,allocation.device_bytes);EXPECT_EQ(run.allocations().device_allocations,allocation.device_allocations);
        EXPECT_EQ(run.host_payload_bytes(),host);
    };
    unchanged();EXPECT_NO_THROW(writer.WriteFrame(run)); // Fresh accepted readback, never the discarded candidate or a raw host fallback.
    EXPECT_NO_THROW(writer.FinishPrefix(run,0,"Numerical rejection of the next attempted interval"));unchanged();
    const auto manifest=ReadJson(dir.path/"manifest.json");EXPECT_FALSE(manifest["horizon_complete"].GetBool());
    EXPECT_EQ(manifest["accepted_epoch"].GetUint64(),1u);EXPECT_EQ(manifest["saved_frames"].GetUint64(),2u);
    EXPECT_EQ(ParseCsvLedgerSegments(manifest[kCsvLedgerSegmentsField])[0].interval_count,1u);
    EXPECT_FALSE(std::filesystem::exists(dir.path/"accepted-000002.fields.json"));
    const auto index=ReadBounded(dir.path/"accepted-frames.csv",WallFrameIndexBytes);EXPECT_EQ(std::count(index.begin(),index.end(),'\n'),3);
    const auto frame=ReadJson(dir.path/"accepted-000001.fields.json");EXPECT_EQ(frame["accepted_epoch"].GetUint64(),1u);
    EXPECT_EQ(frame["contact"]["attempt"].GetUint64(),before.wall.diagnostics.attempt);
    EXPECT_EQ(frame["diagnostics"]["shells"]["qeph"]["attempt"].GetUint64(),before.diagnostics.shells.qeph.attempt);
    const auto row=ReadBounded(dir.path/"accepted-intervals.csv",kArtifactFileCap);EXPECT_EQ(std::count(row.begin(),row.end(),'\n'),2);
}
TEST_F(SourceAssemblyWallArtifactLive, OptionalForceStageArchivesOnlyCommittedActualReadbacks) {
    using Access=cases::source_assembly_dynamics::SourceAssemblyDynamicsTestAccess;
    dynamics::SourceAssemblyWallCase observed;auto config=Configuration();config.observe_force_stage=true;
    ASSERT_TRUE(observed.Initialize(prepared::WallAssembly(),PreparedWall(),config));
    Directory dir;auto request=Request();request.steps=64;request.frame_every=32;
    SourceAssemblyWallArtifacts writer(dir.path.string(),observed,request);writer.WriteFrame(observed);
    EXPECT_EQ(observed.accepted_force_stage(),nullptr);
    std::uint64_t attempt32=0;
    for(unsigned i=0;i<64;++i) {
        const auto base=observed.owner()->accepted();const auto step=observed.Step();ASSERT_TRUE(step)<<step.message;
        ASSERT_NE(observed.accepted_force_stage(),nullptr);writer.RecordInterval(base,observed);
        if(i==31) {
            const auto before=*observed.accepted_force_stage();const auto stamp=observed.owner()->accepted();attempt32=before.attempt;
            const auto rejected=Access::RejectLate(observed,Access::Fault::LastWallFace);
            ASSERT_EQ(rejected.status,dynamics::Status::ComponentFailure)<<rejected.message;
            ASSERT_NE(observed.accepted_force_stage(),nullptr);const auto& after=*observed.accepted_force_stage();
            EXPECT_TRUE(fe::trial_identity::SameStamp(stamp,observed.owner()->accepted()));
            EXPECT_EQ(after.attempt,before.attempt);EXPECT_EQ(after.enclosing_epoch,before.enclosing_epoch);
            EXPECT_EQ(Bits(after.phase.force_time),Bits(before.phase.force_time));
            EXPECT_EQ(Bits(after.native_total),Bits(before.native_total));
            EXPECT_EQ(Bits(after.effective_total),Bits(before.effective_total));
            EXPECT_EQ(Bits(after.replacement),Bits(before.replacement));
        }
        if((i+1)%32==0)writer.WriteFrame(observed);
    }
    writer.Finish(observed,0);
    EXPECT_TRUE(ReadJson(dir.path/"configuration.json")["observe_force_stage"].GetBool());
    EXPECT_TRUE(ReadJson(dir.path/"accepted-000000.fields.json")["force_stage_kinetic"].IsNull());
    const auto retried=ReadJson(dir.path/"accepted-000032.fields.json");
    EXPECT_EQ(retried["force_stage_kinetic"]["attempt"].GetUint64(),attempt32);
    EXPECT_FALSE(std::filesystem::exists(dir.path/"accepted-000033.fields.json"));
    const auto final=ReadJson(dir.path/"accepted-000064.fields.json");const auto& f=final["force_stage_kinetic"];
    const auto& sample=*observed.accepted_force_stage();
    EXPECT_EQ(f["base_epoch"].GetUint64(),63u);EXPECT_EQ(f["enclosing_epoch"].GetUint64(),64u);
    EXPECT_EQ(f["attempt"].GetUint64(),sample.attempt);EXPECT_EQ(Bits(f["native_total_J"].GetDouble()),Bits(sample.native_total));
    EXPECT_EQ(Bits(f["effective_total_J"].GetDouble()),Bits(sample.effective_total));
    EXPECT_EQ(Bits(f["replacement_J"].GetDouble()),Bits(sample.replacement));
    EXPECT_EQ(Bits(f["phase"]["force_time_s"].GetDouble()),Bits(sample.phase.force_time));
    EXPECT_EQ(f["source"]["member_count"].GetUint64(),76u);
}
} // namespace crash::output::assembly::test
