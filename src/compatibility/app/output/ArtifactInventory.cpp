#include "ArtifactInventory.h"
#include <algorithm>

namespace crash::output {
ArtifactInventory::ArtifactInventory(std::filesystem::path directory,std::size_t total_cap)
    : directory_(std::move(directory)),total_cap_(total_cap) {
    Require(total_cap_>0&&total_cap_<=kArtifactMaximumTotalCap,"Invalid artifact aggregate capacity");
}
void ArtifactInventory::Add(const std::string& name, std::size_t component_cap) {
    Require(!name.empty() && name.size()<=255 &&
            name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_.")==std::string::npos &&
            name!="." && name!=".." && name!="manifest.json" && name!="manifest.pending.json" && name!="failure.json",
            "Invalid artifact inventory basename");
    Require(entries_.size()<kArtifactInventoryCap &&
            std::none_of(entries_.begin(),entries_.end(),[&](const Entry& e){return e.file==name;}),
            "Duplicate or over-capacity artifact inventory");
    const auto path=directory_/name;
    Require(std::filesystem::symlink_status(path).type()==std::filesystem::file_type::regular,
            "Artifact inventory requires a regular file");
    const auto data=ReadBounded(path,std::min(component_cap,kArtifactFileCap));
    Require(data.size()<=total_cap_-bytes_,"Artifact aggregate byte cap exceeded");
    Entry staged{name,Sha256(data),data.size()};
    entries_.push_back(std::move(staged)); bytes_+=data.size();
}
void ArtifactInventory::AppendTo(Document& doc) const {
    Require(!doc.HasMember("artifacts"),"Duplicate artifact inventory member");
    Value values(rapidjson::kArrayType);
    for (const auto& e:entries_) {
        Value item(rapidjson::kObjectType),name(e.file.c_str(),doc.GetAllocator()),hash(e.hash.c_str(),doc.GetAllocator());
        item.AddMember("file",name,doc.GetAllocator()); item.AddMember("sha256",hash,doc.GetAllocator());
        item.AddMember("bytes",Value().SetUint64(e.bytes),doc.GetAllocator()); values.PushBack(item,doc.GetAllocator());
    }
    doc.AddMember("artifacts",values,doc.GetAllocator());
}
} // namespace crash::output
