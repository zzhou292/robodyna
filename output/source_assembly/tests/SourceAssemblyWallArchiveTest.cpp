#include "SourceAssemblyWallFieldTestSupport.h"
#include "output/source_assembly/SourceAssemblyWallSequence.h"
#include <fstream>
#include <limits>

namespace crash::output::assembly::test {
namespace {
struct TempDirectory {
    std::filesystem::path path;
    TempDirectory() {const std::string pattern=(std::filesystem::temp_directory_path()/"assembly-wall-archive-XXXXXX").string();
        std::vector<char> chars(pattern.begin(),pattern.end());chars.push_back(0);const auto* made=::mkdtemp(chars.data());Require(made,"Temporary archive directory failed");path=made;}
    ~TempDirectory() {std::error_code e;std::filesystem::remove_all(path,e);}
};
Document Manifest(const ArtifactInventory& inventory) {
    Document d;d.SetObject();String(d,"schema",WallArtifactSchema);String(d,"kind",WallArtifactKind);String(d,"status","completed");
    // This fixture qualifies file-close mechanics only, not a replayable case.
    String(d,"scope","synthetic file publication unit test only");inventory.AppendTo(d);return d;
}
}
TEST(SourceAssemblyWallArchive, ForecastAccountsForActualLedgerAndRejectsCapsBeforeAnyDirectoryWork) {
    auto r=Request();const auto source=source::PinnedYarisSixPartInventory().bytes;const auto wall=PreparedWall().placed_wall()->source_manifest()->size();
    const auto p=PlanWallArchive(r,source,wall);EXPECT_EQ(p.frame_capacity,66u);EXPECT_EQ(p.intervals.interval_count,256u);
    EXPECT_EQ(p.intervals.column_count,39u);EXPECT_LT(p.forecast_bytes,r.limits.total_bytes);
    EXPECT_GE(p.forecast_bytes,p.intervals.total_bytes+source+wall+66*(WallFieldBytes+WallMeshBytes+WallObjBytes));
    EXPECT_TRUE(SameCsvLedgerPlan(p.intervals,PlanCsvLedger("accepted-intervals.csv",WallIntervalHeader,256,39*26)));
    for(unsigned bad=0;bad<6;++bad) {auto q=r;
        if(bad==0)q.limits.total_bytes=p.forecast_bytes-1;if(bad==1)q.limits.frames=65;
        if(bad==2)q.limits.file_bytes=WallFieldBytes-1;if(bad==3)q.limits.total_bytes=WallArchiveTotalCap+1;
        if(bad==4)q.frame_every=0;if(bad==5)q.steps=0;
        EXPECT_THROW(PlanWallArchive(q,source,wall),std::runtime_error);
    }
    r.steps=536871;r.frame_every=20000; // About8ms fits sparse frame budget but exceeds inherited eight-ledger-segment cap.
    EXPECT_THROW(PlanWallArchive(r,source,wall),std::runtime_error);
    r.steps=1000000000;r.frame_every=UINT32_MAX;EXPECT_THROW(PlanWallArchive(r,source,wall),std::runtime_error);
}
TEST(SourceAssemblyWallArchive, DoubledAggregateAllowsDenserFramesWithExactBudgetAdmission) {
    auto r=Request();r.steps=32768;r.frame_every=160;
    const auto source=source::PinnedYarisSevenPartInventory().bytes;
    const auto wall=PreparedWall().placed_wall()->source_manifest()->size();
    const auto plan=PlanWallArchive(r,source,wall);
    EXPECT_EQ(r.limits.total_bytes,std::size_t{2}*1024*1024*1024);
    EXPECT_EQ(plan.frame_capacity,207u);
    EXPECT_GT(plan.forecast_bytes,kArtifactExtendedTotalCap);
    EXPECT_LE(plan.forecast_bytes,WallArchiveTotalCap);
    EXPECT_TRUE(SameCsvLedgerPlan(plan.intervals,
        PlanCsvLedger("accepted-intervals.csv",WallIntervalHeader,r.steps,WallIntervalColumns*26)));
    r.limits.total_bytes=kArtifactExtendedTotalCap;
    EXPECT_THROW(PlanWallArchive(r,source,wall),std::runtime_error);
    r.limits.total_bytes=plan.forecast_bytes;
    EXPECT_EQ(PlanWallArchive(r,source,wall).forecast_bytes,plan.forecast_bytes);
    --r.limits.total_bytes;EXPECT_THROW(PlanWallArchive(r,source,wall),std::runtime_error);
    r.limits.total_bytes=WallArchiveTotalCap+1;
    EXPECT_THROW(PlanWallArchive(r,source,wall),std::runtime_error);
    // The aggregate change allocates no payload and does not enlarge each file.
    TempDirectory dir;ArtifactInventory inventory(dir.path,WallArchiveTotalCap);
    EXPECT_EQ(inventory.bytes(),0u);
    EXPECT_THROW(ArtifactInventory(dir.path,WallArchiveTotalCap+1),std::runtime_error);
    EXPECT_EQ(kArtifactFileCap,32u*1024*1024);
    WriteBytes(dir.path/"payload.txt","complete\n");inventory.Add("payload.txt");
    const auto manifest=Manifest(inventory);
    EXPECT_THROW(wall_files::PublishManifest(dir.path,manifest,WallArchiveTotalCap+1),std::runtime_error);
    EXPECT_FALSE(std::filesystem::exists(dir.path/"manifest.json"));
    EXPECT_NO_THROW(wall_files::PublishManifest(dir.path,manifest,WallArchiveTotalCap));
    EXPECT_TRUE(std::filesystem::exists(dir.path/"manifest.json"));
}
TEST(SourceAssemblyWallArchive, ExactSequenceRejectsWrongBaseDuplicateSkippedCadenceAndNonterminalPrefix) {
    WallFields f;auto r=Request();r.steps=4;r.frame_every=2;WallArchiveSequence s{r,f.stamp,0,0,4,false};
    const auto one=Next(f.stamp);EXPECT_THROW(s.CheckInterval(f.stamp,one),std::runtime_error);
    EXPECT_FALSE(s.CheckFrame(f.stamp));s.AcceptFrame(f.stamp,false);EXPECT_THROW(s.CheckFrame(f.stamp),std::runtime_error);
    auto wrong=f.stamp;++wrong.rigid_groups.source_instance_id;EXPECT_THROW(s.CheckInterval(wrong,one),std::runtime_error);
    EXPECT_NO_THROW(s.CheckInterval(f.stamp,one));s.AcceptInterval(one);EXPECT_THROW(s.CheckInterval(f.stamp,one),std::runtime_error);
    const auto two=Next(one);s.CheckInterval(one,two);s.AcceptInterval(two);const auto three=Next(two);
    EXPECT_THROW(s.CheckInterval(two,three),std::runtime_error);EXPECT_FALSE(s.CheckFrame(two));s.AcceptFrame(two,false);
    s.CheckInterval(two,three);s.AcceptInterval(three);EXPECT_TRUE(s.CheckFrame(three));s.AcceptFrame(three,true);
    EXPECT_NO_THROW(s.CheckClose(three,true));EXPECT_THROW(s.CheckClose(three,false),std::runtime_error);
    EXPECT_THROW(s.CheckInterval(three,Next(three)),std::runtime_error);
    EXPECT_THROW(s.CheckFrame(three),std::runtime_error);
}
TEST(SourceAssemblyWallArchive, BoundedSerializationAndWrongSchemaDoNotCreateCompletionMarker) {
    TempDirectory dir;Document d;d.SetObject();String(d,"message",std::string(1024,'x'));
    EXPECT_THROW(wall_files::WriteJsonBounded(dir.path/"over.json",d,64),std::runtime_error);
    EXPECT_FALSE(std::filesystem::exists(dir.path/"over.json"));
    WriteBytes(dir.path/"payload.txt","complete\n");ArtifactInventory inventory(dir.path);inventory.Add("payload.txt");
    auto m=Manifest(inventory);m["kind"].SetString("source_part_wall",m.GetAllocator());
    EXPECT_THROW(wall_files::PublishManifest(dir.path,m,kArtifactExtendedTotalCap),std::runtime_error);
    EXPECT_FALSE(std::filesystem::exists(dir.path/"manifest.json"));EXPECT_FALSE(std::filesystem::exists(dir.path/"manifest.pending.json"));
}
TEST(SourceAssemblyWallArchive, TruncatedClosedArtifactCannotAcquireCompletionAndGoodInventoryPublishesLast) {
    TempDirectory broken;WriteBytes(broken.path/"payload.txt","complete\n");ArtifactInventory inv(broken.path);inv.Add("payload.txt");auto bad=Manifest(inv);
    {std::ofstream f(broken.path/"payload.txt",std::ios::trunc);f<<"cut";}
    EXPECT_THROW(wall_files::PublishManifest(broken.path,bad,kArtifactExtendedTotalCap),std::runtime_error);
    EXPECT_FALSE(std::filesystem::exists(broken.path/"manifest.json"));EXPECT_FALSE(std::filesystem::exists(broken.path/"manifest.pending.json"));
    TempDirectory good;WriteBytes(good.path/"payload.txt","complete\n");ArtifactInventory inventory(good.path);inventory.Add("payload.txt");auto m=Manifest(inventory);
    EXPECT_NO_THROW(wall_files::PublishManifest(good.path,m,kArtifactExtendedTotalCap));EXPECT_TRUE(std::filesystem::exists(good.path/"manifest.json"));
    EXPECT_FALSE(std::filesystem::exists(good.path/"manifest.pending.json"));EXPECT_EQ(ReadBounded(good.path/"payload.txt",64),"complete\n");
    EXPECT_THROW(wall_files::PublishManifest(good.path,m,kArtifactExtendedTotalCap),std::runtime_error);
}
} // namespace crash::output::assembly::test
