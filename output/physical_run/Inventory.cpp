#include "Metadata.h"
#include <algorithm>
namespace crash::output::physical_run {
std::vector<records::RecordFile> Inventory(const std::filesystem::path& root,std::size_t cap) {
    Require(cap && cap<=records::FullRunByteCap && std::filesystem::symlink_status(root).type()==std::filesystem::file_type::directory,
        "Invalid physical archive directory/cap");
    std::vector<std::string> names;
    for(const auto& entry:std::filesystem::recursive_directory_iterator(root)) {
        const auto status=entry.symlink_status();
        const auto name=entry.path().lexically_relative(root).generic_string();
        if(status.type()==std::filesystem::file_type::directory) {
            Require(name=="arrays","Unexpected physical archive directory");continue;
        }
        Require(status.type()==std::filesystem::file_type::regular,"Physical archive has nonregular entries");
        if(name=="manifest.json")continue;
        Require(names.size()<kArtifactInventoryCap,"Physical inventory count exceeds cap");
        arrays::CheckRelativeName(name);names.push_back(name);
    }
    std::sort(names.begin(),names.end());
    std::vector<records::RecordFile> result;std::size_t total=0;
    for(const auto& name:names) {
        const auto path=arrays::CheckedPath(root,name,true);const auto n=std::filesystem::file_size(path);
        Require(n<=kArtifactFileCap && n<=cap-total,"Physical archive exceeds complete byte cap");
        const auto data=ReadBounded(path,n);result.push_back({name,Sha256(data),static_cast<std::size_t>(n)});total+=n;
    }
    return result;
}
void CheckInventory(const std::filesystem::path& root,const records::RecordFile& file,const Manifest& m,std::size_t cap) {
    Require(file.file=="manifest.json" && file.bytes<=MetadataCap && file.bytes<=cap,"Physical manifest extent differs");
    const auto actual=Inventory(root,cap-file.bytes);
    Require(actual.size()==m.inventory.size(),"Physical inventory count differs");
    for(std::size_t i=0;i<actual.size();++i)Require(actual[i].file==m.inventory[i].file &&
        actual[i].sha256==m.inventory[i].sha256 && actual[i].bytes==m.inventory[i].bytes,"Physical archive inventory differs");
}
} // namespace crash::output::physical_run
