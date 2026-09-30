#include "Support.h"
#include "output/full_shell/tests/FileWriteLimit.h"

namespace crash::output::full_shell::activity::test {
TEST(ParentActivity, AlteredMetadataPayloadPaddingAndTruncationRemainUnpublished) {
    const auto c=Sized(65);const auto s=base::Frame().stamp;const std::vector<std::uint8_t> flags(65,1);
    const auto accepted=ActivityRecord::Create(c,Input(c,s,flags),s);
    for(unsigned fault=0;fault<12;++fault) {
        base::Directory dir;const auto file=WriteActivity(dir.path,"frame",accepted);auto doc=Metadata(dir,file);
        if(fault==0)doc["identity"]["owner"].SetUint64(9);
        if(fault==1)doc["point_layout_sha256"].SetString(std::string(64,'c').c_str(),doc.GetAllocator());
        if(fault==2)doc["parents"].SetUint64(64);
        if(fault==3)doc["phase"].SetString("trial",doc.GetAllocator());
        if(fault==4)doc["attempt"].SetUint64(99);
        if(fault==5)doc["activity"]["dtype"].SetString("float64",doc.GetAllocator());
        if(fault==6)doc["activity"]["shape"][0].SetUint64(1);
        if(fault==7)doc["activity"]["file"].SetString(file.file.c_str(),doc.GetAllocator());
        if(fault>=8) {
            auto bytes=ReadBounded(dir.path/"frame.activity.bin",16);
            if(fault==8) {bytes.back()=char(128);WordBytes(dir,doc,bytes);} // Rehash late unused bit.
            if(fault==9) {bytes[8]=0;base::Overwrite(dir.path/"frame.activity.bin",bytes);} // Last valid bit, stale hash.
            if(fault==10) {bytes.pop_back();base::Overwrite(dir.path/"frame.activity.bin",bytes);}
            if(fault==11)doc["schema"].SetString(FrameSchema,doc.GetAllocator());
        }
        const auto altered=Rewrite(dir,file,doc);
        EXPECT_THROW(ReadActivity(dir.path,c,altered,s),std::runtime_error)<<fault;
        EXPECT_EQ(accepted.words().back(),1);EXPECT_EQ(accepted.active_count(),65);
    }
}
TEST(ParentActivity, DeclarationRejectsForeignContextAndWrongStateSemantics) {
    const auto c=base::MakeContext();base::Directory dir;
    const auto file=WriteDeclaration(dir.path,"parent-activity.json",c);
    auto p=base::Parents();std::swap(p[0],p[1]);const auto reordered=Context::Create(c.identity(),c.nodes(),p.data(),p.size(),c.fixed_dt());
    EXPECT_THROW(ReadDeclaration(dir.path,reordered,file),std::runtime_error);
    auto doc=Metadata(dir,file);doc["active_value"].SetDouble(.8);const auto changed=Rewrite(dir,file,doc);
    EXPECT_THROW(ReadDeclaration(dir.path,c,changed),std::runtime_error);
    auto bad=changed;bad.sha256[0]=bad.sha256[0]=='a'?'b':'a';EXPECT_THROW(ReadDeclaration(dir.path,c,bad),std::runtime_error);
}
TEST(ParentActivity, LateDestinationAndIoFailureKeepCompletionMetadataAbsent) {
    const auto c=base::MakeContext();const auto s=base::Frame().stamp;const std::vector<std::uint8_t> flags{1,0,1,0};
    const auto record=ActivityRecord::Create(c,Input(c,s,flags),s);
    {base::Directory dir;output::WriteBytes(dir.path/"frame.activity.json","occupied");
     EXPECT_THROW(WriteActivity(dir.path,"frame",record),std::runtime_error);
     EXPECT_FALSE(std::filesystem::exists(dir.path/"frame.activity.bin"));}
    {base::Directory dir;full_shell::test::FileSizeLimit limit(4);
     EXPECT_THROW(WriteActivity(dir.path,"frame",record),std::runtime_error);
     EXPECT_FALSE(std::filesystem::exists(dir.path/"frame.activity.json"));
     EXPECT_FALSE(std::filesystem::exists(dir.path/"manifest.json"));}
    base::Directory retry;const auto file=WriteActivity(retry.path,"frame",record);
    EXPECT_EQ(ReadActivity(retry.path,c,file,s).words(),record.words());
}
} // namespace crash::output::full_shell::activity::test
