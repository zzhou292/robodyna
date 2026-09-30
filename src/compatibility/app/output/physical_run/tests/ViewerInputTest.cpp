#include "Support.h"
#include "../ViewerInput.h"
namespace crash::output::physical_run::test {
namespace {
ViewerInput Input() {
    ViewerInput in;
    in.archive_directory="archive";
    in.manifest={"manifest.json",Sha256("manifest"),8};
    in.mapping_sha256=Sha256("mapping");
    in.source.canonical_root="/unused/original";
    in.source.canonical_manifest={"manifest.json",Sha256("canonical"),2632394};
    in.source.scope_report={"scope.json",Sha256("scope"),13175122};
    in.source.source_member={"model.key",Sha256("member"),42846753};
    in.source.tire_policy="omit_original_tire_shells";
    in.source.units={"t","mm","s",1000,.001,1};
    return in;
}
}
TEST(PhysicalViewerInput, ExactSourceReceiptRoundTripAndExternalHashAuthority) {
    ft::Directory directory;
    const auto in=Input();
    const auto f=WriteViewerInput(directory.path,"viewer.json",in);
    const auto actual=ReadViewerInput(directory.path,f);
    EXPECT_EQ(actual.manifest.sha256,in.manifest.sha256);
    EXPECT_EQ(actual.source.source_member.bytes,42846753u);
    EXPECT_TRUE(actual.source.canonical_root.empty());
    EXPECT_TRUE(records::source::SameUnits(actual.source.units,in.source.units));
    EXPECT_EQ(actual.mapping_sha256,in.mapping_sha256);
    EXPECT_THROW(WriteViewerInput(directory.path,"viewer.json",in),std::exception);
    EXPECT_THROW(ViewerArchivePath(directory.path,actual),std::exception);
    std::filesystem::create_directory(directory.path/"archive");
    EXPECT_EQ(ViewerArchivePath(directory.path,actual),directory.path/"archive");
    auto changed=ViewerInputDocument(in);
    changed["mapping_sha256"].SetString(Sha256("foreign").c_str(),changed.GetAllocator());
    const auto other=WriteDocument(directory.path,"changed.json",changed,ViewerInputByteCap);
    ft::Overwrite(directory.path/f.file,ReadFile(directory.path,other,ViewerInputByteCap));
    EXPECT_THROW(ReadViewerInput(directory.path,f),std::exception);
}
TEST(PhysicalViewerInput, StrictSourceIdentityPathsAndInclusiveCap) {
    auto in=Input();
    auto d=ViewerInputDocument(in);
    d["source_authority"]["source_member_bytes"].SetUint64(1);
    EXPECT_THROW(ParseViewerInput(d),std::exception);
    for(const auto* name:{"..","../archive","a/b","/archive",""}) {
        in.archive_directory=name;
        EXPECT_THROW(ViewerInputDocument(in),std::exception);
    }
    in=Input();in.source.source_member.bytes=64u<<20;
    EXPECT_NO_THROW(ViewerInputDocument(in));
    ++in.source.source_member.bytes;
    EXPECT_THROW(ViewerInputDocument(in),std::exception);
    in=Input();ft::Directory directory;
    std::filesystem::create_directory(directory.path/"actual");
    std::filesystem::create_directory_symlink(directory.path/"actual",directory.path/"archive");
    EXPECT_THROW(ViewerArchivePath(directory.path,in),std::exception);
}
} // namespace crash::output::physical_run::test
