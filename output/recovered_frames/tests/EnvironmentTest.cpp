#include "../Internal.h"
#include "../Environment.h"
#include "output/physical_run/RunArchive.h"
#include "output/physical_run/tests/EnvironmentMeshFixture.h"
#include <algorithm>

namespace crash::output::recovered_frames::test {
// Existing record-only recovery fixture from ValuesTest.cpp; no complete
// source bundle, interval ledger or physical owner is fabricated by this suite.
Description Example();
namespace fixture=physical_run::test;
namespace ft=full_shell::test;
namespace {
run::Configuration EnvironmentConfiguration(const records::Context& context) {
    auto config=fixture::Config(context);
    config.environment=true;
    config.request=run::MakeEnvironmentRequest(context,4,.5,3).archive;
    return config;
}
Description EnvironmentDescription(const fixture::DeclaredMesh& mesh) {
    auto description=Example();
    description.mapping_sha256=mesh.receipt.source_mapping_sha256;
    description.environment=mesh.receipt;
    description.files.insert(description.files.end(),mesh.receipt.files.begin(),mesh.receipt.files.end());
    return description;
}
run::WallReceipt LegacyWall(const records::Context& context) {
    run::WallReceipt wall;
    wall.source_instance_id=context.identity().source_instance;
    wall.source_mapping_sha256=context.identity().source_mapping_sha256;
    wall.wall_binding_id=91;
    for(std::size_t i=0;i<wall.files.size();++i)wall.files[i]={run::WallFiles[i],Sha256("legacy"),6};
    return wall;
}
auto InspectAndRead(const fixture::DeclaredMesh& mesh,const run::Configuration& config) {
    const auto receipt=InspectEnvironment(mesh.directory.path,config);
    return run::ReadEnvironmentArtifacts(mesh.directory.path,receipt,mesh.source,mesh.context);
}
Description EnvironmentInventory(const fixture::DeclaredMesh& mesh) {
    Description description;
    description.environment=mesh.receipt;
    description.files.assign(mesh.receipt.files.begin(),mesh.receipt.files.end());
    std::sort(description.files.begin(),description.files.end(),[](const auto& a,const auto& b){return a.file<b.file;});
    return description;
}
std::size_t PayloadBytes(const std::vector<records::RecordFile>& files) {
    std::size_t bytes=0;for(const auto& file:files)bytes+=file.bytes;return bytes;
}
}
TEST(RecoveredEnvironment, LegacyDescriptorsKeepV1AndNeverAcquireEnvironmentClaims) {
    auto legacy=Example();
    auto document=Encode(legacy);
    EXPECT_EQ(array_json::Text(document["schema"]),Schema);
    EXPECT_FALSE(document.HasMember("environment_wall"));
    EXPECT_FALSE(Decode(document).environment);
    legacy.wall=LegacyWall(fixture::Context());
    document=Encode(legacy);
    EXPECT_EQ(array_json::Text(document["schema"]),Schema);
    ASSERT_TRUE(document.HasMember("wall_case"));
    const auto loaded=Decode(document);
    EXPECT_TRUE(loaded.wall);EXPECT_FALSE(loaded.environment);
    EXPECT_FALSE(document["horizon_complete"].GetBool());
    EXPECT_FALSE(document.HasMember("accepted_intervals"));
    EXPECT_FALSE(document.HasMember("segments"));
}
TEST(RecoveredEnvironment, V2RoundtripRequiresEnvironmentAndRejectsWallOrSchemaSubstitution) {
    fixture::DeclaredMesh mesh;auto description=EnvironmentDescription(mesh);
    const auto document=Encode(description);
    EXPECT_EQ(array_json::Text(document["schema"]),EnvironmentSchema);
    EXPECT_TRUE(document.HasMember("environment_wall"));EXPECT_FALSE(document.HasMember("wall_case"));
    const auto loaded=Decode(document);
    ASSERT_TRUE(loaded.environment);EXPECT_FALSE(loaded.wall);
    EXPECT_EQ(loaded.environment->source_instance_id,mesh.receipt.source_instance_id);
    EXPECT_EQ(loaded.environment->wall_binding_id,mesh.receipt.wall_binding_id);
    EXPECT_EQ(loaded.environment->part_id,mesh.receipt.part_id);
    EXPECT_EQ(loaded.environment->source_mapping_sha256,mesh.receipt.source_mapping_sha256);
    EXPECT_EQ(loaded.frames.front().stamp.epoch,description.frames.front().stamp.epoch);
    for(std::size_t i=0;i<3;++i) {
        EXPECT_EQ(loaded.environment->files[i].file,mesh.receipt.files[i].file);
        EXPECT_EQ(loaded.environment->files[i].bytes,mesh.receipt.files[i].bytes);
        EXPECT_EQ(loaded.environment->files[i].sha256,mesh.receipt.files[i].sha256);
    }
    EXPECT_EQ(array_json::Text(document["interval_ledger"]),"unavailable_after_interruption");
    EXPECT_FALSE(document["horizon_complete"].GetBool());
    EXPECT_FALSE(document.HasMember("accepted_intervals"));EXPECT_FALSE(document.HasMember("segments"));
    Document changed;changed.CopyFrom(document,changed.GetAllocator());
    changed["schema"].SetString(Schema,changed.GetAllocator());EXPECT_THROW(Decode(changed),std::exception);
    changed.CopyFrom(document,changed.GetAllocator());changed.RemoveMember("environment_wall");
    EXPECT_THROW(Decode(changed),std::exception);
    changed=Encode(Example());changed["schema"].SetString(EnvironmentSchema,changed.GetAllocator());
    EXPECT_THROW(Decode(changed),std::exception);
    changed.CopyFrom(document,changed.GetAllocator());changed["horizon_complete"].SetBool(true);
    EXPECT_THROW(Decode(changed),std::exception);
    description.wall=LegacyWall(mesh.context);EXPECT_THROW(Encode(description),std::exception);
}
TEST(RecoveredEnvironment, ConfigurationAndReceiptPresenceStayMutuallyExclusive) {
    fixture::DeclaredMesh mesh;const auto description=EnvironmentDescription(mesh);
    auto config=EnvironmentConfiguration(mesh.context);
    EXPECT_NO_THROW(CheckStaticProfiles(config,description));
    config.environment=false;EXPECT_THROW(CheckStaticProfiles(config,description),std::exception);
    config.environment=true;config.wall=true;EXPECT_THROW(CheckStaticProfiles(config,description),std::exception);
    config.wall=false;auto missing=description;missing.environment.reset();
    EXPECT_THROW(CheckStaticProfiles(config,missing),std::exception);
    auto mixed=description;mixed.wall=LegacyWall(mesh.context);
    EXPECT_THROW(CheckStaticProfiles(config,mixed),std::exception);
    auto legacy=Example();legacy.mapping_sha256=mesh.context.identity().source_mapping_sha256;
    auto old=fixture::Config(mesh.context);EXPECT_NO_THROW(CheckStaticProfiles(old,legacy));
    old.wall=true;legacy.wall=LegacyWall(mesh.context);EXPECT_NO_THROW(CheckStaticProfiles(old,legacy));
}
TEST(RecoveredEnvironment, InspectorUsesExactThreeFilesAndExistingTypedMeshAuthority) {
    fixture::DeclaredMesh mesh;const auto config=EnvironmentConfiguration(mesh.context);
    const auto receipt=InspectEnvironment(mesh.directory.path,config);
    EXPECT_EQ(receipt.source_instance_id,mesh.receipt.source_instance_id);
    EXPECT_EQ(receipt.source_mapping_sha256,mesh.receipt.source_mapping_sha256);
    EXPECT_EQ(receipt.wall_binding_id,mesh.receipt.wall_binding_id);EXPECT_EQ(receipt.part_id,mesh.receipt.part_id);
    for(std::size_t i=0;i<3;++i) {
        EXPECT_EQ(receipt.files[i].file,mesh.receipt.files[i].file);
        EXPECT_EQ(receipt.files[i].sha256,mesh.receipt.files[i].sha256);
        EXPECT_EQ(receipt.files[i].bytes,mesh.receipt.files[i].bytes);
    }
    const auto actual=InspectAndRead(mesh,config);
    ASSERT_EQ(actual->GetNumVertices(),4u);ASSERT_EQ(actual->GetNumTriangles(),2u);
    for(unsigned i=0;i<4;++i)for(unsigned k=0;k<3;++k)
        EXPECT_EQ(Bits(actual->GetCoordsVertices()[i][k]),Bits(array_json::Real(mesh.document["reference_m"][i][k])));
    EXPECT_FALSE(std::filesystem::exists(mesh.directory.path/"manifest.json"));
    EXPECT_FALSE(std::filesystem::exists(mesh.directory.path/"frame-index.json"));
}
TEST(RecoveredEnvironment, MissingOrChangedCompanionsRejectAndRepairWithoutReplacingMesh) {
    fixture::DeclaredMesh mesh;const auto config=EnvironmentConfiguration(mesh.context);
    const auto published=InspectAndRead(mesh,config);
    for(const auto& file:mesh.receipt.files) {
        const auto original=run::ReadFile(mesh.directory.path,file,run::EnvironmentFileCap);
        std::filesystem::remove(mesh.directory.path/file.file);
        EXPECT_THROW(InspectAndRead(mesh,config),std::exception);
        ft::Overwrite(mesh.directory.path/file.file,original+"tamper");
        EXPECT_THROW(InspectAndRead(mesh,config),std::exception);
        EXPECT_EQ(Bits(published->GetCoordsVertices()[0].x()),Bits(2.));
        ft::Overwrite(mesh.directory.path/file.file,original);
        EXPECT_NO_THROW(InspectAndRead(mesh,config));
    }
}
TEST(RecoveredEnvironment, ChangedSourceAndMappingCannotBorrowAnotherEnvironment) {
    fixture::DeclaredMesh mesh;const auto accepted=InspectAndRead(mesh,EnvironmentConfiguration(mesh.context));
    auto config=EnvironmentConfiguration(mesh.context);++config.identity.source_instance;
    EXPECT_THROW(InspectAndRead(mesh,config),std::exception);
    config=EnvironmentConfiguration(mesh.context);config.identity.source_mapping_sha256=Sha256("another mapping");
    EXPECT_THROW(InspectAndRead(mesh,config),std::exception);
    Document changed;changed.CopyFrom(mesh.document,changed.GetAllocator());
    changed["vehicle_canonical_sha256"].SetString(Sha256("another vehicle").c_str(),changed.GetAllocator());
    mesh.Replace(std::move(changed),0);
    EXPECT_THROW(InspectAndRead(mesh,EnvironmentConfiguration(mesh.context)),std::exception);
    EXPECT_EQ(accepted->GetNumTriangles(),2u);
}
TEST(RecoveredEnvironment, ExactEnvironmentInventoryAndCopyCapsRejectUnownedArtifacts) {
    fixture::DeclaredMesh mesh;const auto description=EnvironmentInventory(mesh);
    const auto bytes=PayloadBytes(description.files);
    EXPECT_NO_THROW(CheckInventory(mesh.directory.path,description,bytes,false));
    EXPECT_THROW(CheckInventory(mesh.directory.path,description,bytes-1,false),std::exception);
    auto missing=description;missing.files.pop_back();
    EXPECT_THROW(CheckInventory(mesh.directory.path,missing,bytes,false),std::exception);
    WriteBytes(mesh.directory.path/"unowned-environment.json","extra");
    EXPECT_THROW(CheckInventory(mesh.directory.path,description,bytes+5,false),std::exception);
    std::filesystem::remove(mesh.directory.path/"unowned-environment.json");
    ft::Directory destination;
    EXPECT_THROW(CopyFiles(mesh.directory.path,destination.path,description.files,run::MetadataCap+bytes-1),std::exception);
    EXPECT_TRUE(std::filesystem::is_empty(destination.path));
    EXPECT_NO_THROW(CopyFiles(mesh.directory.path,destination.path,description.files,run::MetadataCap+bytes));
    EXPECT_NO_THROW(CheckInventory(destination.path,description,bytes,false));
    EXPECT_FALSE(std::filesystem::exists(destination.path/DescriptorFilename));
    for(const auto& file:description.files)
        EXPECT_EQ(run::ReadFile(destination.path,file,run::EnvironmentFileCap),run::ReadFile(mesh.directory.path,file,run::EnvironmentFileCap));
}
TEST(RecoveredEnvironment, EnvironmentStorageIsChargedAndHostReserveBoundaryRemainsExact) {
    const auto context=fixture::Context();const auto legacy=fixture::Config(context);
    const auto environment=EnvironmentConfiguration(context);
    Limits limits;
    const auto before=Budget(context,legacy,limits),after=Budget(context,environment,limits);
    // The small fixture's sample and environment workspace are both below the
    // existing file-copy workspace; the retained mesh is additional live data.
    EXPECT_EQ(after-before,run::EnvironmentRetainedBytes);
    limits.host_bytes=limits.source.host_bytes+(128u<<20);
    EXPECT_NO_THROW(Budget(context,environment,limits));
    --limits.host_bytes;EXPECT_THROW(Budget(context,environment,limits),std::exception);
}
} // namespace crash::output::recovered_frames::test
