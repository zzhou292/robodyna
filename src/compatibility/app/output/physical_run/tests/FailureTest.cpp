#include "Support.h"
#include "../../full_shell/tests/FileWriteLimit.h"
namespace crash::output::physical_run::test {
TEST(PhysicalRunFailure, InvalidInputRetriesButLateChunkWriteFailureIsSticky) {
    ft::Directory dir;const auto c=Context();const Profile profile{false,true};
    IntervalWriter writer(dir.path,c,profile,4,624,4096);
    auto bad=Row(c,2);EXPECT_THROW(writer.Append(bad),std::exception);EXPECT_EQ(writer.sequence().last.epoch,0u);
    writer.Append(Row(c,1));
    WriteBytes(dir.path/"interval-0.reals.bin","occupied late destination");
    EXPECT_THROW(writer.Append(Row(c,2)),std::exception);EXPECT_TRUE(writer.failed());
    EXPECT_FALSE(std::filesystem::exists(dir.path/"interval-0.integers.bin"));
    EXPECT_THROW(writer.Finish(),std::exception);
    EXPECT_FALSE(std::filesystem::exists(dir.path/"manifest.json"));
}
TEST(PhysicalRunFailure, ActualShortWriteRetainsIncompleteEvidenceAndCannotFinish) {
    ft::Directory dir;const auto c=Context();
    IntervalWriter writer(dir.path,c,{false,true},4,624,4096);
    writer.Append(Row(c,1));
    {
        ft::FileSizeLimit limit(1);
        EXPECT_THROW(writer.Append(Row(c,2)),std::exception);
    }
    EXPECT_TRUE(writer.failed());
    EXPECT_TRUE(std::filesystem::exists(dir.path/"interval-0.integers.bin"));
    EXPECT_FALSE(std::filesystem::exists(dir.path/"interval-0.reals.bin"));
    EXPECT_THROW(writer.Finish(),std::exception);
    EXPECT_FALSE(std::filesystem::exists(dir.path/"manifest.json"));
}
TEST(PhysicalRunFailure, RehashedLateActivityAndExtraFilesRejectWithoutReplacingPriorRecords) {
    ft::Directory dir;const auto c=Context();const auto config=Config(c);
    const auto index=WriteRun(dir.path,c,config,3);
    EXPECT_NO_THROW(ValidateRecords(dir.path,c,config,index,64u<<20));
    const auto& last=index.frames.back();
    const auto original=ReadFile(dir.path,last.activity,MetadataCap);
    auto corrupt=original;corrupt.back()^=1;ft::Overwrite(dir.path/last.activity.file,corrupt);
    EXPECT_THROW(ValidateRecords(dir.path,c,config,index,64u<<20),std::exception);
    ft::Overwrite(dir.path/last.activity.file,original);
    EXPECT_NO_THROW(ValidateRecords(dir.path,c,config,index,64u<<20));
    auto wrong=index;
    auto metadata=array_json::Parse(original,MetadataCap);
    metadata["attempt"].SetUint64(last.stamp.attempt+1);
    std::filesystem::remove(dir.path/last.activity.file);
    wrong.frames.back().activity=WriteDocument(dir.path,last.activity.file,metadata,MetadataCap);
    EXPECT_THROW(ValidateRecords(dir.path,c,config,wrong,64u<<20),std::exception);
    ft::Overwrite(dir.path/last.activity.file,original);
    const auto inventory=Inventory(dir.path,records::TotalByteCap);
    Manifest manifest;manifest.inventory=inventory;
    WriteBytes(dir.path/"manifest.json","{}");
    const records::RecordFile record{"manifest.json",Sha256("{}"),2};
    EXPECT_NO_THROW(CheckInventory(dir.path,record,manifest,records::TotalByteCap));
    WriteBytes(dir.path/"extra.bin","unexpected");
    EXPECT_THROW(CheckInventory(dir.path,record,manifest,records::TotalByteCap),std::exception);
}
TEST(PhysicalRunFailure, CompleteHostCapsRejectBeforeMissingPathRead) {
    const auto c=Context();const Profile p{};
    EXPECT_THROW(IntervalWriter("/absent-physical-run",c,p,100,624,1),std::exception);
    EXPECT_THROW(ReadIntervals("/absent-physical-run",c,p,100,0,{},624,1,{}),std::exception);
    EXPECT_THROW(Inventory("/absent-physical-run",records::FullRunByteCap+1),std::exception);
    ReplayLimits limits;limits.host_bytes=1;
    EXPECT_THROW(Replay::Open("/absent-physical-run",{}, {},std::string(64,'a'),limits),std::exception);
}
TEST(PhysicalRunFailure, EvenRehashedOuterInventoryCannotAdmitUnreferencedCompanions) {
    ft::Directory dir;const auto c=Context();const auto config=Config(c);
    const auto index=WriteRun(dir.path,c,config,3);
    Manifest manifest;
    manifest.configuration=WriteDocument(dir.path,"configuration.json",ConfigurationDocument(config),MetadataCap);
    manifest.index=WriteDocument(dir.path,"frame-index.json",IndexDocument(config,index),MetadataCap);
    manifest.activity_declaration=records::activity::WriteDeclaration(dir.path,"parent-activity.json",c);
    // This isolated inventory test supplies the already-validated source list.
    // Actual replay separately authenticates the complete original bundle first.
    Document source;source.SetObject();source.AddMember("files",Value(rapidjson::kArrayType),source.GetAllocator());
    manifest.source=WriteDocument(dir.path,"source.bundle.json",source,MetadataCap);
    manifest.inventory=Inventory(dir.path,records::TotalByteCap);
    EXPECT_NO_THROW(CheckReferencedInventory(dir.path,c,index,manifest));
    WriteBytes(dir.path/"unknown-case.json","{}");
    manifest.inventory=Inventory(dir.path,records::TotalByteCap);
    EXPECT_THROW(CheckReferencedInventory(dir.path,c,index,manifest),std::exception);
    std::filesystem::remove(dir.path/"unknown-case.json");
    manifest.inventory=Inventory(dir.path,records::TotalByteCap);
    EXPECT_NO_THROW(CheckReferencedInventory(dir.path,c,index,manifest));
}
} // namespace crash::output::physical_run::test
