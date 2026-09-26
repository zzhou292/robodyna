#include "Internal.h"

namespace crash::output::recovered_frames {
namespace {
void Destination(const std::filesystem::path& source,const std::filesystem::path& destination) {
    Require(std::filesystem::symlink_status(source).type()==std::filesystem::file_type::directory &&
        std::filesystem::symlink_status(destination).type()==std::filesystem::file_type::directory &&
        std::filesystem::is_empty(destination),"Recovery needs an original directory and a real empty new destination");
    const auto a=std::filesystem::canonical(source),b=std::filesystem::canonical(destination);
    auto i=a.begin(),j=b.begin();
    while(i!=a.end() && j!=b.end() && *i==*j) {++i;++j;}
    Require(i!=a.end(),"Recovery destination must be outside the preserved original archive");
}
}
records::RecordFile Recover(const std::filesystem::path& source,const std::filesystem::path& destination,
        const records::source::SourceInputs& authority,const std::string& digest,
        const std::string& reason,Limits limits) {
    Destination(source,destination);
    Require(!reason.empty() && reason.size()<=4096,"Recovery reason is missing or too long");
    Require(!std::filesystem::exists(source/"manifest.json") &&
        !std::filesystem::exists(source/DescriptorFilename),"Recovery expects interrupted unpublished originals");
    // This first recovery profile addresses an entirely lost buffered ledger.
    // Do not silently discard a durable interval stream that needs its own audit.
    for(const auto& file:std::filesystem::directory_iterator(source))
        Require(file.path().filename().string().rfind("interval-",0)!=0,"Recovery source has durable interval evidence");
    Description description;
    description.source=authority;description.mapping_sha256=digest;description.reason=reason;
    description.configuration=InspectFile(source,"configuration.json",run::MetadataCap);
    description.source_bundle=InspectFile(source,"source.bundle.json",records::source::BundleMetadataByteCap);
    description.activity_declaration=InspectFile(source,"parent-activity.json",records::activity::MetadataByteCap);
    const auto config=run::ReadConfiguration(array_json::Parse(
        run::ReadFile(source,description.configuration,run::MetadataCap),run::MetadataCap));
    Require(!config.environment,"Declared environment recovery requires a separately qualified descriptor");
    if(config.wall) {
        run::WallReceipt wall;
        wall.source_instance_id=config.identity.source_instance;
        wall.source_mapping_sha256=digest;
        for(std::size_t i=0;i<wall.files.size();++i)wall.files[i]=InspectFile(source,run::WallFiles[i],run::WallFileCap);
        const auto setup=array_json::Parse(run::ReadFile(source,wall.files[6],run::WallFileCap),run::WallFileCap);
        Require(setup.IsObject() && setup.HasMember("wall_binding_id"),"Recovery wall setup lacks binding identity");
        wall.wall_binding_id=array_json::UInt(setup["wall_binding_id"]);
        description.wall=std::move(wall);
    }
    const auto loaded=LoadSource(source,description,limits);
    description.frames=ScanFrames(source,loaded.context,loaded.configuration);
    if(description.wall) {
        std::optional<run::WallComposition> composition;
        run::ReadWallArtifacts(source,*description.wall,loaded.mapping.source().data(),loaded.context,&composition);
        run::CheckWallBeamObservation(loaded.configuration.profile.beam18,composition);
    }
    description.files=ReferencedFiles(source,loaded.context,description);
    const auto document=Encode(description);
    // All source/sample/identity/alias/cap checks precede creation of payload
    // directories. Copy rechecks original byte hashes; no original writes occur.
    Require(loaded.configuration.request.total_byte_cap>=run::MetadataCap,"Recovery metadata exceeds total cap");
    std::size_t total=run::MetadataCap;
    for(const auto& file:description.files) {
        Require(file.bytes<=loaded.configuration.request.total_byte_cap-total,"Recovery total archive cap exceeded");
        total+=file.bytes;
    }
    std::filesystem::create_directory(destination/"arrays");
    if(description.wall)std::filesystem::create_directory(destination/"wall");
    CopyFiles(source,destination,description.files,loaded.configuration.request.total_byte_cap);
    CheckInventory(destination,description,loaded.configuration.request.total_byte_cap-run::MetadataCap,false);
    return run::WriteDocument(destination,DescriptorFilename,document,run::MetadataCap);
}
} // namespace crash::output::recovered_frames
