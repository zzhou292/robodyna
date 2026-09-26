#include "Support.h"
#include "../RunArchive.h"
#include "../RunState.h"
namespace crash::output::physical_run::test {
namespace {
EnvironmentReceipt Environment(const records::Context& c) {
    EnvironmentReceipt e;e.source_instance_id=c.identity().source_instance;e.wall_binding_id=91;e.part_id=301;
    e.source_mapping_sha256=c.identity().source_mapping_sha256;
    for(unsigned i=0;i<3;++i)e.files[i]={EnvironmentFiles[i],Sha256("fixture"),7};return e;
}
Manifest ManifestValue(const records::Context& c) {
    Manifest m;m.identity=c.identity();m.forecast_bytes=100000;
    m.configuration={"configuration.json",Sha256("config"),7};m.index={"frame-index.json",Sha256("index"),7};
    m.source={"source.bundle.json",Sha256("source"),7};m.activity_declaration={"parent-activity.json",Sha256("activity"),7};
    m.inventory={m.configuration,m.index,m.source,m.activity_declaration};return m;
}
}
TEST(PhysicalEnvironmentRecords, SeparateProfileKeepsLegacyConfigurationAndWallReceiptClosed) {
    const auto context=Context();auto legacy=Config(context);
    const auto old=ConfigurationDocument(legacy);
    EXPECT_FALSE(old.HasMember("environment_wall"));EXPECT_FALSE(ReadConfiguration(old).environment);
    auto config=legacy;config.environment=true;config.request=MakeEnvironmentRequest(context,4,.5,3).archive;
    const auto doc=ConfigurationDocument(config);
    EXPECT_EQ(array_json::Text(doc["schema"]),"robo_dyna.physical_run_configuration.v4");
    EXPECT_TRUE(ReadConfiguration(doc).environment);EXPECT_FALSE(ReadConfiguration(doc).wall);
    config.wall=true;
    EXPECT_THROW(ConfigurationDocument(config),std::exception);
    config.wall=false;config.profile.native_contact=true;
    EXPECT_THROW(ConfigurationDocument(config),std::exception);
    auto bad=ConfigurationDocument(ReadConfiguration(doc));bad["environment_wall"].SetString("canonical-wall",bad.GetAllocator());
    EXPECT_THROW(ReadConfiguration(bad),std::exception);
    bad=ConfigurationDocument(ReadConfiguration(doc));bad.RemoveMember("schema");
    EXPECT_THROW(ReadConfiguration(bad),std::exception);
}
TEST(PhysicalEnvironmentRecords, CompleteThreeFileReservationHasNoCanonicalWallSubstitution) {
    const auto context=Context();auto request=MakeEnvironmentRequest(context,4,.5,3);
    EXPECT_EQ(request.archive.static_files.size(),6u);
    EXPECT_NO_THROW(detail::ValidateRequest(request.archive,{},false,true));
    EXPECT_THROW(detail::ValidateRequest(request.archive,{},false,false),std::exception);
    EXPECT_THROW(detail::ValidateRequest(request.archive,{},true,true),std::exception);
    request.archive.static_files.back().file="wall/original-canonical-wall.manifest.json";
    EXPECT_THROW(detail::ValidateRequest(request.archive,{},false,true),std::exception);
    auto receipt=Environment(context);const auto restored=ReadEnvironmentDocument(EnvironmentDocument(receipt));
    EXPECT_EQ(restored.part_id,301u);EXPECT_EQ(restored.files.size(),3u);
    receipt.files[2].file=receipt.files[0].file;
    EXPECT_THROW(EnvironmentDocument(receipt),std::exception);
}
TEST(PhysicalEnvironmentRecords, ManifestDispatchCannotHideOrMixStaticSourceProfiles) {
    const auto context=Context();auto m=ManifestValue(context);
    EXPECT_EQ(array_json::Text(ManifestDocument(m)["schema"]),Schema);
    m.environment=Environment(context);
    for(const auto& file:m.environment->files)m.inventory.push_back(file);
    const auto doc=ManifestDocument(m);const auto loaded=ReadManifest(doc);
    ASSERT_TRUE(loaded.environment);EXPECT_FALSE(loaded.wall);EXPECT_EQ(loaded.environment->part_id,301u);
    auto missing=ManifestDocument(m);missing.RemoveMember("environment_wall");
    EXPECT_THROW(ReadManifest(missing),std::exception);
    auto wrong=ManifestDocument(m);wrong["schema"].SetString(Schema,wrong.GetAllocator());
    EXPECT_THROW(ReadManifest(wrong),std::exception);
    WallReceipt old{context.identity().source_instance,1,context.identity().source_mapping_sha256,{}};
    for(unsigned i=0;i<7;++i)old.files[i]={WallFiles[i],Sha256("legacy"),6};m.wall=old;
    EXPECT_THROW(ManifestDocument(m),std::exception);
}
}
