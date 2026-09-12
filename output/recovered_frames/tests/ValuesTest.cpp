#include "../Internal.h"
#include "output/physical_run/tests/Support.h"
#include "output/physical_run/SourceInputFields.h"
#include "output/physical_run/ViewerInput.h"

namespace crash::output::recovered_frames::test {
namespace fixture=physical_run::test;
namespace ft=full_shell::test;
Description Example() {
    Description d;
    const records::RecordFile f{"canonical.json",std::string(64,'a'),123};
    d.source.canonical_manifest=f;
    d.source.scope_report={"scope.json",std::string(64,'b'),456};
    d.source.source_member={"original.key",std::string(64,'c'),43u<<20};
    d.source.tire_policy="strict_original";
    d.source.units={"tonne","mm","s",1000,.001,1};
    d.mapping_sha256=std::string(64,'d');d.reason="Hard guard interrupted recording";
    d.configuration={"configuration.json",std::string(64,'e'),1};
    d.source_bundle={"source.bundle.json",std::string(64,'f'),1};
    d.activity_declaration={"parent-activity.json",std::string(64,'a'),1};
    d.frames.push_back({{}, {"frame-0.frame.json",std::string(64,'b'),1},
        {"frame-0.activity.json",std::string(64,'c'),1}});
    d.files={d.configuration,d.source_bundle,d.activity_declaration};
    return d;
}
TEST(RecoveredSamples, DistinctSchemaRoundtripAndNoCompletionClaim) {
    const auto value=Example();
    auto document=Encode(value);
    const auto actual=Decode(document);
    EXPECT_EQ(actual.reason,value.reason);
    EXPECT_EQ(actual.source.source_member.bytes,43u<<20);
    EXPECT_EQ(actual.mapping_sha256,value.mapping_sha256);
    EXPECT_EQ(actual.frames.size(),1u);
    EXPECT_FALSE(document.HasMember("accepted_intervals"));
    EXPECT_FALSE(document.HasMember("segments"));
    document["horizon_complete"].SetBool(true);
    EXPECT_THROW(Decode(document),std::exception);
    document["horizon_complete"].SetBool(false);
    document["schema"].SetString(physical_run::Schema,document.GetAllocator());
    EXPECT_THROW(Decode(document),std::exception);
}
TEST(RecoveredSamples, SharedSourceReceiptExtractionPreservesLegacyFields) {
    auto value=Example();
    run::ViewerInput input{"archive",{"manifest.json",std::string(64,'e'),123},value.source,value.mapping_sha256};
    const auto document=run::ViewerInputDocument(input);
    const auto parsed=run::ParseViewerInput(document);
    EXPECT_EQ(parsed.source.source_member.sha256,input.source.source_member.sha256);
    EXPECT_EQ(parsed.source.source_member.bytes,input.source.source_member.bytes);
    EXPECT_EQ(parsed.mapping_sha256,input.mapping_sha256);
    EXPECT_EQ(parsed.archive_directory,"archive");
}
TEST(RecoveredSamples, ScanKeepsExactCompleteCadenceAndStopsAtMissingPair) {
    ft::Directory root;
    const auto context=fixture::Context();const auto config=fixture::Config(context);
    fixture::Frame(root.path,context,{});
    fixture::Frame(root.path,context,fixture::Row(context,2).stamp);
    auto frames=ScanFrames(root.path,context,config);
    ASSERT_EQ(frames.size(),2u);
    EXPECT_EQ(frames.back().stamp.epoch,2u);
    std::filesystem::remove(root.path/"frame-2.activity.json");
    frames=ScanFrames(root.path,context,config);
    EXPECT_EQ(frames.size(),1u);
    std::filesystem::remove(root.path/"frame-0.frame.json");
    EXPECT_THROW(ScanFrames(root.path,context,config),std::exception);
}
TEST(RecoveredSamples, CorruptCompletePayloadAndReorderedStampsReject) {
    ft::Directory root;
    const auto context=fixture::Context();const auto config=fixture::Config(context);
    fixture::Frame(root.path,context,{});
    fixture::Frame(root.path,context,fixture::Row(context,2).stamp);
    auto frames=ScanFrames(root.path,context,config);
    auto bytes=ReadBounded(root.path/"frame-2.plastic.bin",1024);
    bytes.back()^=1;ft::Overwrite(root.path/"frame-2.plastic.bin",bytes);
    EXPECT_THROW(ScanFrames(root.path,context,config),std::exception);
    frames[1].stamp.epoch=3;
    EXPECT_THROW(CheckFrames(context,config,frames),std::exception);
}
TEST(RecoveredSamples, CopyCapPreflightAndLateHashFailurePreserveOriginal) {
    ft::Directory source,destination;
    WriteBytes(source.path/"a.bin","1234");WriteBytes(source.path/"b.bin","56789");
    std::vector<records::RecordFile> files{InspectFile(source.path,"a.bin",10),InspectFile(source.path,"b.bin",10)};
    EXPECT_THROW(CopyFiles(source.path,destination.path,files,run::MetadataCap+8),std::exception);
    EXPECT_TRUE(std::filesystem::is_empty(destination.path));
    ft::Overwrite(source.path/"b.bin","56780");
    EXPECT_THROW(CopyFiles(source.path,destination.path,files,run::MetadataCap+9),std::exception);
    EXPECT_TRUE(std::filesystem::exists(destination.path/"a.bin"));
    EXPECT_FALSE(std::filesystem::exists(destination.path/DescriptorFilename));
    EXPECT_EQ(ReadBounded(source.path/"a.bin",10),"1234");
    EXPECT_EQ(ReadBounded(source.path/"b.bin",10),"56780");
}
TEST(RecoveredSamples, RecordedCadenceAndBoundedWorkspaceStayIndependentOfLedger) {
    const auto context=fixture::Context();const auto config=fixture::Config(context);
    const auto bytes=Budget(context,config,{});
    EXPECT_LT(bytes,512u<<20);
    Limits zero;zero.host_bytes=0;
    EXPECT_THROW(Budget(context,config,zero),std::exception);
    Limits too_large;too_large.host_bytes=(512u<<20)+1;
    EXPECT_THROW(Budget(context,config,too_large),std::exception);
    EXPECT_TRUE(config.profile.structural_limit); // Original declaration retained;
    EXPECT_STREQ(Schema,"robo_dyna.recovered_sampled_review.v1"); // no row reconstructed.
}
} // namespace crash::output::recovered_frames::test
