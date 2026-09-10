#pragma once
#include "AcceptedReplayData.h"
#include <cstdlib>
#include <stdexcept>
namespace crash::output::test_support {
// Test-only immutable-payload clone. Replacement unlinks its directory entry
// first, so a corruption probe never writes through the original hard link.
class ModifiedReplayBundle {
  public:
    std::filesystem::path directory;
    explicit ModifiedReplayBundle(const std::filesystem::path& source) {
        namespace fs=std::filesystem;auto pattern=(fs::temp_directory_path()/"replay-bundle-XXXXXX").string();
        std::vector<char> name(pattern.begin(),pattern.end());name.push_back(0);const auto made=::mkdtemp(name.data());
        Require(made,"Cannot create replay test clone");directory=made;
        for(const auto& entry:fs::directory_iterator(source))if(entry.is_regular_file()) {
            std::error_code error;fs::create_hard_link(entry.path(),directory/entry.path().filename(),error);
            if(error)fs::copy_file(entry.path(),directory/entry.path().filename());
        }
    }
    ~ModifiedReplayBundle(){std::error_code error;std::filesystem::remove_all(directory,error);}
    Document Read(const std::string& file)const{return replay_detail::Json(ReadBounded(directory/file,32*1024*1024));}
    void Replace(const std::string& file,const Document& d){std::filesystem::remove(directory/file);WriteJson(directory/file,d);}
    void ReplaceBytes(const std::string& file,const std::string& bytes){std::filesystem::remove(directory/file);WriteBytes(directory/file,bytes);}
    void Rehash(const std::string& file) {
        auto manifest=Read("manifest.json");const auto bytes=ReadBounded(directory/file,32*1024*1024);bool found=false;
        for(auto& item:manifest["artifacts"].GetArray())if(file==item["file"].GetString()){
            item["sha256"].SetString(Sha256(bytes).c_str(),manifest.GetAllocator());item["bytes"].SetUint64(bytes.size());found=true;}
        Require(found,"Mutated replay test file was not inventoried");Replace("manifest.json",manifest);
    }
    std::string Frame(std::uint64_t epoch)const {
        const auto manifest=Read("manifest.json");for(const auto& item:manifest["artifacts"].GetArray()) {
            const std::string name=item["file"].GetString();if(name.size()>12&&name.substr(name.size()-12)==".fields.json") {
                const auto f=Read(name);if(f["accepted_epoch"].GetUint64()==epoch)return name;}}
        throw std::runtime_error("Required epoch was not saved in the replay test bundle");
    }
};
}
