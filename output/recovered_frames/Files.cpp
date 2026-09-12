#include "Internal.h"
#include <algorithm>

namespace crash::output::recovered_frames {
records::RecordFile InspectFile(const std::filesystem::path& root,const std::string& name,std::size_t cap) {
    const auto path=arrays::CheckedPath(root,name,true);
    const auto size=std::filesystem::file_size(path);
    Require(size && size<=cap && size<=kArtifactFileCap,"Recovery record exceeds bounded file cap");
    const auto bytes=ReadBounded(path,static_cast<std::size_t>(size));
    return {name,Sha256(bytes),static_cast<std::size_t>(size)};
}
std::vector<records::RecordFile> ReferencedFiles(const std::filesystem::path& root,
        const records::Context& context,const Description& description) {
    auto result=run::ReferencedSampleFiles(root,context,description.source_bundle,
        description.activity_declaration,description.frames,description.wall);
    for(const auto& file:result)Require(file.file!=description.configuration.file,
        "Recovery configuration aliases a payload");
    result.push_back(description.configuration);
    std::sort(result.begin(),result.end(),[](const auto& a,const auto& b){return a.file<b.file;});
    return result;
}
void CheckInventory(const std::filesystem::path& root,const Description& description,std::size_t cap,bool closed) {
    Require(!std::filesystem::exists(root/"manifest.json"),"Recovered samples cannot carry a normal run manifest");
    auto files=run::Inventory(root,cap,bool(description.wall));
    if(closed) {
        const auto found=std::find_if(files.begin(),files.end(),[](const auto& file){return file.file==DescriptorFilename;});
        Require(found!=files.end(),"Recovery descriptor is missing");
        files.erase(found);
    }
    Require(files.size()==description.files.size(),"Recovered inventory contains missing or unowned files");
    for(std::size_t i=0;i<files.size();++i)Require(files[i].file==description.files[i].file &&
        files[i].bytes==description.files[i].bytes && files[i].sha256==description.files[i].sha256,
        "Recovered inventory differs from descriptor");
}
void CopyFiles(const std::filesystem::path& source,const std::filesystem::path& destination,
        const std::vector<records::RecordFile>& files,std::size_t cap) {
    Require(cap<=records::FullRunByteCap && cap>=run::MetadataCap,"Recovery total byte cap is invalid");
    std::size_t total=run::MetadataCap;
    for(const auto& file:files) {
        run::FileDocument(file);
        Require(file.bytes<=cap-total,"Recovered payloads exceed selected total cap");
        total+=file.bytes;
    }
    // Maximum copy buffer is one already bounded file (32 MiB). Copies are
    // independent immutable bytes, never writable hard links into the original.
    for(const auto& file:files) {
        const auto data=run::ReadFile(source,file,kArtifactFileCap);
        WriteBytes(arrays::CheckedPath(destination,file.file,false),data);
    }
}
} // namespace crash::output::recovered_frames
